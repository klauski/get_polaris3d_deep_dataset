#include "ImuGrabber.h"

#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <iomanip>
#include <iostream>

#include <pthread.h>

#include <sys/mman.h>

#include <dlib/serialize.h>

#define USE_ASIO 0
#if USE_ASIO
#include <asio.hpp>
#endif

#include <glog/logging.h>

#include <vsys.h>

#include "icm20602.h"

namespace v{

// Note that both functions should be declared and the namespace should be same with its type. 
static void serialize(const ImuAxis6_40& item, std::ostream& out)
{
	for (int i = 0; i < sizeof(ImuAxis6_40) / sizeof(ImuAxis6); i++) {
		dlib::serialize(item.d[i].time, out);
		dlib::serialize(item.d[i].gyr, out);
		dlib::serialize(item.d[i].acc, out);
	}
}

static void deserialize(ImuAxis6_40& item, std::istream& out)
{
	for (int i = 0; i < sizeof(ImuAxis6_40) / sizeof(ImuAxis6); i++) {
		dlib::deserialize(item.d[i].time, out);
		dlib::deserialize(item.d[i].gyr, out);
		dlib::deserialize(item.d[i].acc, out);
	}
}

namespace io_node{

IMUGrabber::IMUGrabber() 
	: opipe_(250), brg_status_(4)
	, bInit_(false), bExit_(false)
	, fdev_(icm20602_open("/dev/spidev1.0"))
	, brg_(dlib::listen_on_port(ETHERNET_PORT_IMUDATA_FRM), dlib::transmit(opipe_), dlib::receive(brg_status_))
{
	if (fdev_ < 0) { LOG(WARNING) << "cannot open icm20602 (spidev1.0)"; return; }
	
	icm20602_init(fdev_);

	pth_.reset(new std::thread(&IMUGrabber::thread, this));
	LOG_IF(FATAL, pth_ == NULL) << "Nullptr is returned after thread creation.";

	// Setup scheduling policy
	int sch_policy = SCHED_FIFO;
	sched_param sch_params;
	sch_params.sched_priority = THREAD_PRIORITY_IMU_GRABBER;
	if(pthread_setschedparam(pth_->native_handle(), sch_policy, &sch_params)) {
		LOG(FATAL) << "Failed to set Thread scheduling : " << std::strerror(errno);
	}

	bInit_ = true;
}

IMUGrabber::~IMUGrabber() {
	pth_->join();

	icm20602_close(fdev_);
}

void IMUGrabber::spin() { while(!bExit_) { usleep(100*1000); } }

#if USE_ASIO

namespace shmem_imu_data {
// Receive session and server for Differential control by keyboard 
class Session
  : public std::enable_shared_from_this<Session>
{
public:
  Session(stream_protocol::socket sock, std::queue<v::ImuAxis6>& q) : socket_(std::move(sock)), q_(q) { }
  void start() { do_read(); }
private:
  void do_read()
  {
    auto self(shared_from_this());
    socket_.async_read_some(asio::buffer((char*)&imu_data_),
        [this, self](std::error_code ec, std::size_t length)
        {
          if (!ec) {
						q_.push(imu_data_);		
            do_read();
          }
        });
  }
  // The socket used to communicate with the client.
  stream_protocol::socket socket_;
  // Buffer used to store data received from the client.
	v::ImuAxis6 imu_data_;
	std::queue<v::ImuAxis6>& q_;
};
class Server
{
public:
  Server(asio::io_context& io_context, const std::string& file)
		: bConn_(false), io_context_(io_context), acceptor_(io_context, stream_protocol::endpoint(file)) { do_accept(); }
	void spin() { io_context_.run(); }
	bool isConn() { return bConn_; }

	std::queue<v::ImuAxis6> q_;

private:
  void do_accept()
  {
    acceptor_.async_accept(
			[this](std::error_code ec, stream_protocol::socket s)
			{
				if (!ec) {
					std::make_shared<Session>(std::move(s), q_)->start();
					bConn_ = true;
					LOG(INFO) << "Sending imu data starts.";
				}
				LOG(INFO) << "Sending imu data stops.";
        do_accept();
      });
  }
	bool bConn_;
  stream_protocol::acceptor acceptor_;
	asio::io_context& io_context_;
};
}/* namespace shared_memory_differential_control_by_keyboard */


void IMUGrabber::thread() {
using asio::local::stream_protocol;

//	bool bConn = false;
//	IMUData_t imuData = {0};
	char buf[256];
//	uint64_t nImus=0;
//	v::ImuAxis6_40 imus;

	while(!bInit_) usleep(10000); // sleep for 10 ms.
	LOG(INFO) << "IMUGrabber thread is launched.";

	// try to setup shared memory interface
	asio::io_context io_context;
	::unlink(".asio_imu_data");
	shmem_imu_data::Server imuServ(io_context, ".asio_imu_data");
	asio::executor_work_guard<asio::io_context::executor_type> work = asio::make_work_guard(io_context);

	imuServ.spin();
	
	work.reset();

	std::unique_ptr<v::chrono::SyncTimer> sync = v::chrono::createSyncTimer();
	sync->arm(0.00025); // 250 us, 0.25 ms
	//TODO: bConn should be re-initialized when disconnected and reconnected
	dlib::bridge_status bs;
	while(!bExit_) {
		// Check bridge status
		if (brg_status_.size() > 0) {
			// Blocked until new status message is arrived.
			brg_status_.dequeue(bs);
			bConn = bs.is_connected;
			if (bConn) LOG(INFO) << "New connection is established from " << bs.foreign_ip << ":" << bs.foreign_port;
			else LOG(INFO) << "The connection is destroyed from " << bs.foreign_ip << ":" << bs.foreign_port;
			// Clear the output pipe for new transmission.
			opipe_.empty();
			nImus = 0;
		}
		// Read and send imu data if client was prepared.
		if (bConn && icm20602_read(fdev_, &imuData)) {
			v::ImuAxis6 imu_data((double)imuData.ts.tv_sec + imuData.ts.tv_nsec/1e9, imuData.gyro, imuData.accl);
			asio::write(s, asio::buffer((char*)&imu_data, sizeof(v::ImuAxis6));
			printf("%2.4lf, ", imu_data.time);
		}

		sync->synchronize();
	}


}
#else
void IMUGrabber::thread() {
	bool bConn = false;
	IMUData_t imuData = {0};
	char buf[256];
	uint64_t nImus=0;
	v::ImuAxis6_40 imus;

	while(!bInit_) usleep(10000); // sleep for 10 ms.
	LOG(INFO) << "IMUGrabber thread is launched.";

	std::unique_ptr<v::chrono::SyncTimer> sync = v::chrono::createSyncTimer();
	sync->arm(0.00025); // 250 us, 0.25 ms
	//TODO: bConn should be re-initialized when disconnected and reconnected
	dlib::bridge_status bs;
	while(!bExit_) {
		// Check bridge status
		if (brg_status_.size() > 0) {
			// Blocked until new status message is arrived.
			brg_status_.dequeue(bs);
			bConn = bs.is_connected;
			if (bConn) LOG(INFO) << "New connection is established from " << bs.foreign_ip << ":" << bs.foreign_port;
			else LOG(INFO) << "The connection is destroyed from " << bs.foreign_ip << ":" << bs.foreign_port;
			// Clear the output pipe for new transmission.
			opipe_.empty();
			nImus = 0;
		}
		// Read and send imu data if client was prepared.
		if (bConn && icm20602_read(fdev_, &imuData)) {
			imus.d[nImus] = v::ImuAxis6((double)imuData.ts.tv_sec + imuData.ts.tv_nsec/1e9, imuData.gyro, imuData.accl);
			if (++nImus == 40) {
//				sprintf(buf, "%2.4lf,%2.4lf,%2.4lf,%2.4lf,%2.4lf,%2.4lf,%2.4lf,%2.4lf,%2.4lf,%2.4lf",
//					imus.d[0].time, imus.d[1].time, imus.d[2].time, imus.d[3].time, imus.d[4].time, 
//					imus.d[5].time, imus.d[6].time, imus.d[7].time, imus.d[8].time, imus.d[9].time);
//				LOG(INFO) << "Imus: " << buf;
				
				opipe_.enqueue(imus);
				nImus = 0;
			}
		}

		sync->synchronize();
	}
}
#endif



}
}






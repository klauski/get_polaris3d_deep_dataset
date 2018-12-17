#include "ControlEmitter.h"

#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <algorithm>

#include <sys/mman.h>

#include <asio.hpp>

#include <glog/logging.h>

#include <opencv2/highgui/highgui.hpp>

#include "ControlUdp.h"
#include "Mocap2Mav/mocap2mav.h"
#include "myRoboClaw/software/driver/roboclaw.hpp"

#include "messageRobotics.pb.h"

using asio::local::stream_protocol;

namespace v{

static void serialize (const v::Control8Ch& item, std::ostream& out)
{
	dlib::serialize(item.chn, out);
}

static void deserialize (v::Control8Ch& item, std::istream& out)
{
	dlib::deserialize(item.chn, out);
}

namespace io_node{

namespace shmem_differential_control_by_keyboard {
// Receive session and server for Differential control by keyboard 
class Session
  : public std::enable_shared_from_this<Session>
{
public:
  Session(stream_protocol::socket sock, std::queue<mDifferentialControl8Ch>& q) : socket_(std::move(sock)), q_(q) { }
  void start() { do_read(); }
private:
  void do_read()
  {
    auto self(shared_from_this());
    socket_.async_read_some(asio::buffer(req_),
        [this, self](std::error_code ec, std::size_t length)
        {
          if (!ec) {
						// deserialize received message. 
						mDifferentialControl8Ch dcmd;
						dcmd.ParseFromArray(req_, length);
						q_.push(dcmd);		
						
            do_read();
          }
        });
  }
  // The socket used to communicate with the client.
  stream_protocol::socket socket_;
  // Buffer used to store data received from the client.
	char req_[64];
	std::queue<mDifferentialControl8Ch>& q_;
};
class Server
{
public:
  Server(asio::io_context& io_context, const std::string& file)
		: bExit_(false), io_context_(io_context), acceptor_(io_context, stream_protocol::endpoint(file)) { do_accept(); }
	void spin() { io_context_.run(); }

	std::queue<mDifferentialControl8Ch> q_;

private:
  void do_accept()
  {
    acceptor_.async_accept(
			[this](std::error_code ec, stream_protocol::socket s)
			{
				if (!ec) std::make_shared<Session>(std::move(s), q_)->start();
        do_accept();
      });
  }
	bool bExit_;
  stream_protocol::acceptor acceptor_;
	asio::io_context& io_context_;
};
}/* namespace shared_memory_differential_control_by_keyboard */

namespace shmem_control_by_planner {
// Receive session and server for Differential control by keyboard 
class Session
  : public std::enable_shared_from_this<Session>
{
public:
  Session(stream_protocol::socket sock, std::queue<mControl8Ch>& q) : socket_(std::move(sock)), q_(q) { }
  void start() { do_read(); }
private:
  void do_read()
  {
    auto self(shared_from_this());
    socket_.async_read_some(asio::buffer(req_),
        [this, self](std::error_code ec, std::size_t length)
        {
          if (!ec) {
						// deserialize received message. 
						mControl8Ch cmd;
						cmd.ParseFromArray(req_, length);
						q_.push(cmd);		
						
            do_read();
          }
        });
  }
  // The socket used to communicate with the client.
  stream_protocol::socket socket_;
  // Buffer used to store data received from the client.
	char req_[64];
	std::queue<mControl8Ch>& q_;
};
class Server
{
public:
  Server(asio::io_context& io_context, const std::string& file)
		: bExit_(false), io_context_(io_context), acceptor_(io_context, stream_protocol::endpoint(file)) { do_accept(); }
	void spin() { io_context_.run(); }

	std::queue<mControl8Ch> q_;

private:
  void do_accept()
  {
    acceptor_.async_accept(
			[this](std::error_code ec, stream_protocol::socket s)
			{
				if (!ec) std::make_shared<Session>(std::move(s), q_)->start();
        do_accept();
      });
  }
	bool bExit_;
  stream_protocol::acceptor acceptor_;
	asio::io_context& io_context_;
};
}/* namespace shared_memory_differential_control_by_keyboard */


////
////
////
ControlEmitter::ControlEmitter(std::shared_ptr<Mocap2Mav> mocap2mav) 
	: ipipe_(250), brg_status_(4), bExit_(false)
	, brg_(dlib::listen_on_port(ETHERNET_PORT_CNTL_ALG_TO), dlib::transmit(ipipe_), dlib::receive(brg_status_))
	, mocap2mav_(mocap2mav)
	, current_yaw_(1500), current_ttt_(1100)
{
//	uart0_.reset(new v::sys_tx2::Uart(UART_PORT_CONTROL));

	pth_.reset(new std::thread(&ControlEmitter::thread, this));
	LOG_IF(FATAL, pth_ == NULL) << "Nullptr is returned after thread creation.";

	// Setup scheduling policy
	int sch_policy = SCHED_FIFO;
	sched_param sch_params;
	sch_params.sched_priority = THREAD_PRIORITY_CTL_EMITTER;
	if(pthread_setschedparam(pth_->native_handle(), sch_policy, &sch_params)) {
		LOG(FATAL) << "Failed to set Thread scheduling : " << std::strerror(errno);
	}
}

ControlEmitter::ControlEmitter(std::shared_ptr<RoboClaw> roboclaw) 
	: ipipe_(250), brg_status_(4), bExit_(false)
	, brg_(dlib::listen_on_port(ETHERNET_PORT_CNTL_ALG_TO), dlib::transmit(ipipe_), dlib::receive(brg_status_))
	, roboclaw_(roboclaw)
	, current_yaw_(1500), current_ttt_(1100)
{
//	uart0_.reset(new v::sys_tx2::Uart(UART_PORT_CONTROL));

	pth_.reset(new std::thread(&ControlEmitter::thread, this));
	LOG_IF(FATAL, pth_ == NULL) << "Nullptr is returned after thread creation.";

	// Setup scheduling policy
	int sch_policy = SCHED_FIFO;
	sched_param sch_params;
	sch_params.sched_priority = THREAD_PRIORITY_CTL_EMITTER;
	if(pthread_setschedparam(pth_->native_handle(), sch_policy, &sch_params)) {
		LOG(FATAL) << "Failed to set Thread scheduling : " << std::strerror(errno);
	}
	// Initialize roboclaw
	if (roboclaw_) {
		int result;
		result = roboclaw_->setSpeedM1M2(0, 0);
		LOG_IF(WARNING, result != 0) << "Fail to stop motors in RoboClaw with error code: " << result;
		result = roboclaw_->resetEncoders();
		LOG_IF(WARNING, result != 0) << "Fail to reset encoders in RoboClaw with error code: " << result;
		PID_t	pidM1, pidM2;
		pidM1.Kp = 0x00400000; pidM1.Ki = 0x00100000; pidM1.Kd = 0x00000000; pidM1.qpps = MOTOR_CTRL_VEL_QPPS;
		pidM2.Kp = 0x00400000; pidM2.Ki = 0x00100000; pidM2.Kd = 0x00000000; pidM2.qpps = MOTOR_CTRL_VEL_QPPS;
		result = roboclaw_->setM1VelocityPID(pidM1);
		LOG_IF(WARNING, result != 0) << "Fail to set PID constants of the velocity controller in RoboClaw with error code: " << result;
		result = roboclaw_->setM2VelocityPID(pidM2);
		LOG_IF(WARNING, result != 0) << "Fail to set PID constants of the velocity controller in RoboClaw with error code: " << result;
	}
}

ControlEmitter::~ControlEmitter() {
	if (roboclaw_) roboclaw_->setSpeedM1M2(0, 0);
	pth_->join();
}


void ControlEmitter::spin() { while(!bExit_) { usleep(100); } }

void ControlEmitter::thread() {
	bool bConn = false;
	v::Control8Ch ctrl;
	LOG(INFO) << "ControlEmitter (" << (roboclaw_ ? "ROBOCLAW" : (mocap2mav_ ? "PX4" : "NONE")) << ") thread is launched.";

	// control by webviewer
	ControlUdp cudp(ETHERNET_ADDR_WEB_VIEWER, ETHERNET_PORT_SRV_RBTMETA, ETHERNET_PORT_CLI_RCSRECV);

	// control by keyboard (library ASIO is used)
  asio::io_context io_ctxt_dcntl;
	::unlink(".asio_control_keyboard");
	shmem_differential_control_by_keyboard::Server dcServ(io_ctxt_dcntl, ".asio_control_keyboard");
	std::thread dcntl_run_thr([] (shmem_differential_control_by_keyboard::Server* dcServ) { dcServ->spin(); }, &dcServ);	
	asio::executor_work_guard<asio::io_context::executor_type> work= asio::make_work_guard(io_ctxt_dcntl);

	// control by planner (library ASIO is used)
  asio::io_context io_ctxt_planner;
	::unlink(".asio_control_planner");
	shmem_control_by_planner::Server dcServPlanner(io_ctxt_planner, ".asio_control_planner");
	std::thread cntl_planner_run_thr([] (shmem_control_by_planner::Server* dcServPlanner) { dcServPlanner->spin(); }, &dcServPlanner);	
	asio::executor_work_guard<asio::io_context::executor_type> work_planner= asio::make_work_guard(io_ctxt_planner);


	std::unique_ptr<v::chrono::SyncTimer> sync = v::chrono::createSyncTimer();
	sync->arm(0.0005); // 500 us, 0.5 ms
	dlib::bridge_status bs;
	while(!bExit_) {
		if (brg_status_.size() > 0) {
			// Blocked until new status message is arrived.
//			LOG(INFO) << "Wait for new connection...";
			brg_status_.dequeue(bs);
			bConn = bs.is_connected;
			if (bConn) LOG(INFO) << "New connection is established from " << bs.foreign_ip << ":" << bs.foreign_port;
			else LOG(INFO) << "The connection is destroyed from " << bs.foreign_ip << ":" << bs.foreign_port;
			// Clear the output pipe for new transmission.
//			ipipe_.empty();
		}
		// Read control message and emit it.
		while (bConn && ipipe_.size() > 0) {
			ipipe_.dequeue(ctrl);
			LOG(INFO) << "yaw: " << ctrl.yaw << ", vel: " << ctrl.throttle;
			//TODO: Define control packet and add uart emitter
//			uart0_->send();
		}
		// Read manual control message and emit it (MAVROS).
		while (cudp.q_.size() > 0) {
			Control8Ch cmd = cudp.q_.front();
			LOG(INFO) << "Emit manual control message: roll (" << cmd.roll << "), throttle (" << cmd.throttle << ")";
			cudp.q_.pop();
			if (mocap2mav_) {
				current_yaw_ = cmd.yaw;
				current_ttt_ = cmd.throttle;
//				mocap2mav_->control_rc(current_yaw_, current_ttt_);
			}
		}
		// Read and emit control command by keyboard
		while (dcServ.q_.size() > 0 ) {
			mDifferentialControl8Ch dcmd = dcServ.q_.front(); dcServ.q_.pop();
			current_ttt_ = std::min((uint32_t)1900, std::max((uint32_t)1100, (uint32_t)((int)current_ttt_ + dcmd.throttle())));
			current_yaw_ = std::min((uint32_t)1900, std::max((uint32_t)1100, (uint32_t)((int)current_yaw_ + dcmd.roll())));
			if (dcmd.throttle() == -1 && dcmd.roll() == -1) {
				current_ttt_ = 1100;
				current_yaw_ = 1500; // reset 
				LOG(INFO) << "reset signal";
			}
			// Emit control command
			if (roboclaw_) {
				// ROBOCLAW
				int speedM1 = (current_yaw_ - 1500.f) / 400.f * 1920;
				int speedM2 = -speedM1;
				speedM1 += (current_ttt_ - 1100.f) / 800.f * 1920;
				speedM2 += (current_ttt_ - 1100.f) / 800.f * 1920;
				float speed = sqrt(speedM1 *speedM1 + speedM2 * speedM2) + 1.0;
				speedM1 = speedM1 / speed * 1920 * 2;
				speedM2 = speedM2 / speed * 1920 * 2;
				LOG(INFO) << "[m] speedM1: " << speedM1 << ", speedM2: " << speedM2;
				roboclaw_->setSpeedM1M2(speedM1, speedM2);
			} else if (mocap2mav_) {
				int sz =mocap2mav_->control_rc(current_yaw_, current_ttt_);
				LOG(INFO) << "[m] yaw: " << current_yaw_ << ", throttle: " << current_ttt_;
			}
		}
		// Read and emit control command by planner
		while (dcServPlanner.q_.size() > 0 ) {
			mControl8Ch cmd  = dcServPlanner.q_.front(); dcServPlanner.q_.pop();
			current_ttt_ = std::min((uint32_t)1900, std::max((uint32_t)1100, (uint32_t)((int)cmd.throttle())));
			current_yaw_ = std::min((uint32_t)1900, std::max((uint32_t)1100, (uint32_t)((int)cmd.roll())));
			// Emit control command
			if (roboclaw_) {
				// ROBOCLAW
				int speedM1 = -(current_yaw_ - 1500.f) / 400.f * 1920;
				int speedM2 = -speedM1;
				speedM1 += (current_ttt_ - 1100.f) / 800.f * 1920;
				speedM2 += (current_ttt_ - 1100.f) / 800.f * 1920;
				float speed = sqrt(speedM1 *speedM1 + speedM2 * speedM2) + 1.0;
				speedM1 = speedM1 / speed * 1920 ;
				speedM2 = speedM2 / speed * 1920 ;
				LOG(INFO) << "[m] speedM1: " << speedM1 << ", speedM2: " << speedM2;
				roboclaw_->setSpeedM1M2(speedM1, speedM2);
			} else if (mocap2mav_) {
				// PX4
				int sz = mocap2mav_->control_rc(current_yaw_, current_ttt_);
				LOG(INFO) << "[m] yaw: " << current_yaw_ << ", throttle: " << current_ttt_;
			}
		}

		sync->synchronize();
	}

	work.reset();
	work_planner.reset();
}



}
}



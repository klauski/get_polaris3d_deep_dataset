#include "ControlUdp.h"

#include <thread>

#include <glog/logging.h>

#include <vsys.h>

using asio::ip::udp;

namespace v{
namespace io_node{

/** @brief  Server mode creation. */
ControlUdp::ControlUdp(short port)
  : op_mode_(SERVER_MODE)
  , bClientAlive_(false), bExit_(false)
{
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	
	socket_.reset(new udp::socket(io_context_, udp::endpoint(udp::v4(), port)));
	// setup recv handler
  srv_recv();
	// launch spin() thread
	hnd_th_.reset(new std::thread([] (ControlUdp* cudp) {
		cudp->spin();
	}, this));
	LOG(INFO) << "ControlUDP (server mode, listen port: " << port << ") is created.";
}

/** @brief  Client mode creation. */
ControlUdp::ControlUdp(std::string dst_addr, short dst_port, short src_port)
  : op_mode_(CLIENT_MODE), bExit_(false)
  , dst_addr_(dst_addr), dst_port_(dst_port), src_port_(src_port)
{
	cli_init();
	// launch keep alive() thread
	cli_th0_.reset(new std::thread(&ControlUdp::cli_sendAlive, this));
	// launch spin() thread
	hnd_th_.reset(new std::thread([] (ControlUdp* cudp) {
		cudp->spin();
	}, this));	
}

ControlUdp::~ControlUdp() {
	if (cli_th0_) cli_th0_->join();
	if (hnd_th_) hnd_th_->join();
}

void ControlUdp::srv_recv()
{
	CHECK(op_mode_ == SERVER_MODE);
	// recv handler: control command     
	socket_->async_receive_from(
      asio::buffer(data_, max_length), sender_endpoint_,
      [this](std::error_code ec, std::size_t bytes_recvd)
      {
        if (!ec && bytes_recvd > 0 && strcmp(data_, "alive") == 0)
        {
        	LOG(INFO) << "#recv: " << bytes_recvd << ", msg: " << data_;
					bClientAlive_ = true;
					t_last_ = v::chrono::tic();
        }
        else
        {
          srv_recv();
        }
      });
}

void ControlUdp::srv_send(mControl8Ch cmd)
{
	CHECK(op_mode_ == SERVER_MODE);
	if (!(bClientAlive_ = checkTimeout())) return; // is client alive?

	char msg[1024];
	cmd.SerializeToArray(msg, cmd.ByteSize());
//	sprintf(msg, "[0:4] %d, %d, %d, %d", cmd.roll(), cmd.pitch(), cmd.throttle(), cmd.yaw());
	std::size_t mlen = std::strlen(msg);
  socket_->async_send_to(
      asio::buffer(msg, mlen), sender_endpoint_,
      [this](std::error_code /*ec*/, std::size_t /*bytes_sent*/)
      {
//        srv_recv();
      });
}

bool ControlUdp::checkTimeout()
{
	if (v::chrono::toc(t_last_) > 5e6)
	{
		return false;
	}
	return true;
}

void ControlUdp::spin()
{
	try {
		asio::error_code ec;
		io_context_.run(ec);
		if (ec) {
			std::cout << "spin????\n";
		}
	} catch(std::exception &ex) {
		std::cout << "spin\n";
	}
}

bool ControlUdp::cli_init() 
{
	try {
		socket_.reset(new udp::socket(io_context_, udp::endpoint(udp::v4(), src_port_)));
	  udp::resolver resolver(io_context_);

	  dst_endpoint_ = resolver.resolve(udp::v4(), dst_addr_.c_str(), std::to_string(dst_port_).c_str());
#if 1
		// setup recv handler
		cli_recv();
#endif
		// read MAC address for identification
		uint8_t mac[6];
		v::filesystem::readMacAddress(mac, "wlan0", &macAddr_);
	//	printf("%02x:%02x:%02x:%02x:%02x:%02x(%lx)\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], macAddr_);
		LOG(INFO) << "Control Server (" << dst_addr_ << ":" << dst_port_ << ") is successfully connected via UDP.";
	} catch (const std::system_error &e) {
		LOG(WARNING) << "Network is unreachable.";
		socket_.reset();
		return false;
	}
}

/** @brief  Client sends alive message periodically. */
void ControlUdp::cli_sendAlive()
{
	mRobotMetadata mRobotMeta;
	mRobotMeta.set_mac_addr(macAddr_);
	mRobotMeta.set_writable(true);
	char msg[512] = {0};
	mRobotMeta.SerializeToArray(msg, mRobotMeta.ByteSizeLong());
	uint64_t nsends = 0;
	while(!bExit_) {
		if (!socket_) {
			cli_init();
		} else {
			try {
			  socket_->send_to(asio::buffer(msg, mRobotMeta.ByteSizeLong()), *dst_endpoint_.begin());
				if (nsends++ % 30 == 0) LOG(INFO) << "send alive " << nsends << " times.";
			} catch (const std::system_error& e) {
				socket_.reset();
			}
		}
		v::chrono::sleep_once(1000); // 1 sec
	}
}

void ControlUdp::cli_recv()
{
	CHECK(op_mode_ == CLIENT_MODE);
	// recv handler: control command 
	try {
	if (socket_) return;
#if 1
	socket_->async_receive_from(
      asio::buffer(data_, max_length), sender_endpoint_,
      [this](std::error_code ec, std::size_t bytes_recvd)
      {
        if (!ec && bytes_recvd > 0)
        {
					// deserialize received message. 
					mControl8Ch cmd;
					cmd.ParseFromArray(data_, bytes_recvd);
					//TODO: serialize/deserialize message.
					Control8Ch cmd1; memset(&cmd1, 0, sizeof(Control8Ch));
					cmd1.roll = cmd.roll();
					cmd1.throttle = cmd.throttle();
					q_.push(cmd1);
        }
        cli_recv();
      });
#endif
	} catch (...) {
		std::cout << "??" << std::endl;
	}
}

}
}

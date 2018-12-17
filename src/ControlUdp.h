#pragma once

#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <queue>
#include <thread>
#include <memory>

#include <asio.hpp>

#include <vsys.h>

#include "messageRobotics.pb.h"

namespace v{
namespace io_node{

class ControlUdp
{
public:
	/** @brief  Server mode creation. */
  ControlUdp(short port = ETHERNET_PORT_SRV_RBTMETA);
	/** @brief  Client mode creation. */
	ControlUdp(std::string dst_addr = ETHERNET_ADDR_WEB_VIEWER, short dst_port = ETHERNET_PORT_SRV_RBTMETA, short src_port = ETHERNET_PORT_CLI_RCSRECV);

	~ControlUdp();

	bool checkTimeout();
	void spin();

  void srv_recv();
  void srv_send(mControl8Ch cmd);

	bool cli_init();
  void cli_recv();

	std::queue<Control8Ch> q_;

private:
	enum { SERVER_MODE, CLIENT_MODE };
	std::string dst_addr_;
	short dst_port_;
	short src_port_;
  std::unique_ptr<asio::ip::udp::socket> socket_;
  asio::ip::udp::endpoint sender_endpoint_;
	asio::ip::udp::resolver::results_type dst_endpoint_;
	int op_mode_;
	uint64_t macAddr_;
	bool bClientAlive_;
	bool bExit_;
	uint64_t t_last_; // last time of server/client checked.
	
  enum { max_length = 10240 };
  char data_[max_length];

	asio::io_context io_context_;
	std::unique_ptr<std::thread> cli_th0_;
	std::unique_ptr<std::thread> hnd_th_;

	void cli_sendAlive();
};

}
}

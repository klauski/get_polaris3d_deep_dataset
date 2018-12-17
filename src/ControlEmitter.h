#pragma once

#include <memory>
#include <thread>

#include <dlib/pipe.h>
#include <dlib/bridge.h>

#include <vsys.h>

#define MOTOR_CONTROL_NONE 1
#define MOTOR_CONTROL_ROBOCLAW 1

class Mocap2Mav;
class RoboClaw;

namespace v{
namespace io_node{

class ControlEmitter {
public:
	ControlEmitter(std::shared_ptr<Mocap2Mav> mocap2mav);
	ControlEmitter(std::shared_ptr<RoboClaw> roboclaw);
	~ControlEmitter();
	void spin();
	void tryExit() { bExit_ = true; }
	
private:
	bool bExit_;
	dlib::pipe<v::Control8Ch> ipipe_;
	dlib::pipe<dlib::bridge_status> brg_status_;
	std::unique_ptr<v::sys_tx2::Uart> uart0_;

	dlib::bridge brg_;
	std::unique_ptr<std::thread> pth_;
	std::shared_ptr<Mocap2Mav> mocap2mav_;
	std::shared_ptr<RoboClaw> roboclaw_;

	uint32_t current_ttt_;
	uint32_t current_yaw_;
	
	void thread();
};

}
}

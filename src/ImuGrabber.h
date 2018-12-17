#pragma once

#include <memory>
#include <thread>

#include <dlib/pipe.h>
#include <dlib/bridge.h>
#include <dlib/threads.h>

#include "icm20602.h"

#include <vsys.h>

namespace v{
namespace io_node{

class IMUGrabber {
public:
	IMUGrabber();
	~IMUGrabber();
	void spin();
	void tryExit() { bExit_ = true;}
	
private:
	bool bInit_;
	bool bExit_;
	int fdev_;			// IMU device
	dlib::pipe<v::ImuAxis6_40> opipe_;
	dlib::pipe<dlib::bridge_status> brg_status_;

	dlib::bridge brg_;
	std::unique_ptr<std::thread> pth_;

	void thread();
};

}
}

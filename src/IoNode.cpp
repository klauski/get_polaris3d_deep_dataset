#include <signal.h>

#include <glog/logging.h>

#include <vsys.h>

#include "ImuGrabber.h"
#include "ControlEmitter.h"

#include "myRoboClaw/software/driver/roboclaw.hpp"
#include "Mocap2Mav/mocap2mav.h"

std::unique_ptr<v::io_node::IMUGrabber> imu;
std::unique_ptr<v::io_node::ControlEmitter> cem;

// Control via RoboClaw
std::shared_ptr<RoboClaw> roboclaw;
// Control via PX4
std::shared_ptr<Mocap2Mav> mocap2mav;

#include <execinfo.h> // for backtrace
#include <dlfcn.h>    // for dladdr
#include <cxxabi.h>   // for __cxa_demangle

#include <X11/Xlib.h> // for XInitThreads

// This function produces a stack backtrace with demangled function & method names.
std::string Backtrace(int skip = 1)
{
    void *callstack[128];
    const int nMaxFrames = sizeof(callstack) / sizeof(callstack[0]);
    char buf[1024];
    int nFrames = backtrace(callstack, nMaxFrames);
    char **symbols = backtrace_symbols(callstack, nFrames);

    std::ostringstream trace_buf;
    for (int i = skip; i < nFrames; i++) {
        printf("%s\n", symbols[i]);

        Dl_info info;
        if (dladdr(callstack[i], &info) && info.dli_sname) {
            char *demangled = NULL;
            int status = -1;
            if (info.dli_sname[0] == '_')
                demangled = abi::__cxa_demangle(info.dli_sname, NULL, 0, &status);
            snprintf(buf, sizeof(buf), "%-3d %*p %s + %zd\n",
                     i, int(2 + sizeof(void*) * 2), callstack[i],
                     status == 0 ? demangled :
                     info.dli_sname == 0 ? symbols[i] : info.dli_sname,
                     (char *)callstack[i] - (char *)info.dli_saddr);
            free(demangled);
        } else {
            snprintf(buf, sizeof(buf), "%-3d %*p %s\n",
                     i, int(2 + sizeof(void*) * 2), callstack[i], symbols[i]);
        }
        trace_buf << buf;
    }
    free(symbols);
    if (nFrames == nMaxFrames)
        trace_buf << "[truncated]\n";
    return trace_buf.str();
}

void handler(int sig) {  
	std::cerr << Backtrace();
  exit(1);
}

void quit_handler(int signal) {
	if (imu) imu->tryExit();
	if (cem) cem->tryExit();
	if (mocap2mav) mocap2mav->quit_handler(signal);
	if (roboclaw) roboclaw->setSpeedM1M2(0, 0);
}

int main(int argc, const char *argv[])
{
	using namespace v::io_node;

	if (!v::filesystem::exdir("log")) v::filesystem::mkdir("log");
	FLAGS_alsologtostderr = true;
	google::SetLogDestination(google::GLOG_INFO, "./log/");
	google::InitGoogleLogging(argv[0]);


	signal(SIGSEGV, handler);   // install our handler

#if MOTOR_CONTROL_NONE
// no output here
#elif MOTOR_CONTROL_ROBOCLAW
	// Initialize roboclaw for packet serial control
	std::shared_ptr<Serial> serial(new Serial(0, 57600));
	roboclaw.reset(new RoboClaw(serial));
	LOG(INFO) << "Packet serial mode is enabled (ROBOCLAW motor contoller)";
#else
	// Initialize mocap2mav for px4 control
	mocap2mav.reset(new Mocap2Mav(false /* disable mocap, TODO: define config */));
  LOG(INFO) << "RC override mode is enabled (PX4)";
#endif

	// Setup interrupt signal handler
	signal(SIGINT, quit_handler);
	// Start
	if (mocap2mav) {
		if (!mocap2mav->start()) {
			LOG(WARNING) << "Error arised during serial port or control interface setup. PX4 control is disabled.";
			mocap2mav.reset();
		}
	}

	imu.reset(new IMUGrabber());
	if (roboclaw) 
		cem.reset(new ControlEmitter(roboclaw));
	else 
		cem.reset(new ControlEmitter(mocap2mav));

	if (mocap2mav) mocap2mav->spin();
	else imu->spin();
	
  return 0;
}





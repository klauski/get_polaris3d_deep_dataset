#include <memory>
#include <utility>

#include <asio.hpp>

#include <glog/logging.h>

#include <vsys.h>

#include "messageRobotics.pb.h"

using asio::local::stream_protocol;

void quit_handler(int signal) {
//	v::term::kbmode(false);
}

int main(void) {
	std::unique_ptr<v::chrono::SyncTimer> sync = v::chrono::createSyncTimer();
	sync->arm(1./2); // 25 Hz
bool bauto = false;
int _thr= 1100;
	try {
	  char req[64];
	  asio::io_context io_context;
	  stream_protocol::socket s(io_context);		
	  s.connect(stream_protocol::endpoint(".asio_control_keyboard"));

//		v::term::kbmode(true);
		signal(SIGINT, quit_handler); // register quit_handler to return normal terminal mode when ctrl+c is pressed.
		printf("Type 'w','s' to control throttle and 'a','d' to control yaw angle.\r\n");
                printf("Type 'i' to control throttle and yaw angle with dronet output\r\n");
		printf("Type <space> to stop robot.\r\n"); 
		printf("Type 'q' or 'Q' to exit.\r\n");
		while(true) {
				mDifferentialControl8Ch dcmd;
#if 0
			int kb = v::term::getch();
			if (kb >= 0) {

				char key = (char)kb;
				switch(key) {
					case 'w': dcmd.set_throttle(+5); break;
					case 's': dcmd.set_throttle(-5); break;
					case 'a': dcmd.set_roll(-5); break;
					case 'd': dcmd.set_roll(+5); break;
					case ' ': dcmd.set_roll(-1); dcmd.set_throttle(-1); break;
                                }
#endif
{
#define FN_TIMESTAMP "/home/nvidia/ws/rpg_public_dronet/Dronet/time_stamp.txt"
  FILE *fd = fopen(FN_TIMESTAMP, "r");
  printf("a");
  if (fd) {
    float steer, pcoll;
    char mm[128];
    fscanf(fd, "%f, %f\n", &steer, &pcoll);
    printf("%f, %f\r\n", steer, pcoll);
    
    if (pcoll > 0.9) {
      printf("stop\n");
      dcmd.set_roll(-1);
//      dcmd.set_throttle(-(_thr-1100));
      dcmd.set_throttle(-1);
      _thr = 1100;
    } else {
      printf("%d  ", _thr);
      if(_thr > 1150) {
        dcmd.set_throttle(0);
//        dcmd.set_roll(steer*50);
      } else {
        dcmd.set_throttle(+10);
        _thr += 10;
//        dcmd.set_roll(steer*50);
      }
    }
    fclose(fd);
    fd = fopen(FN_TIMESTAMP, "w");
    fclose(fd);
  }
}
				if (dcmd.roll() != 0 || dcmd.throttle() != 0) {
					// Serialize and write
					dcmd.SerializeToArray(req, dcmd.ByteSizeLong());
					asio::write(s, asio::buffer(req, dcmd.ByteSizeLong()));
				}
//				if (key == 'q' || key == 'Q') break;
//			}		
			sync->synchronize();
		}
  } 
	catch (std::exception& e)
  {
		LOG(FATAL) << "Exception: " << e.what();
  }
	quit_handler(0);
}

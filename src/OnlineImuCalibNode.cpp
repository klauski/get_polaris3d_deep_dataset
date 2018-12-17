#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <iomanip>
#include <iostream>

#include <sys/types.h>
#include <dirent.h>
#include <signal.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>

#include <glog/logging.h>

#include <vsys.h>

#include "icm20602.h"

#include <ImuToolkit.h>

const std::string DATA_DIR = "./imu_data";

int fdev = -1;
FILE *fd_data = NULL;

/*function... might want it in some class?*/
int getdir (const std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        std::cout << "Error(" << errno << ") opening " << dir << std::endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        files.push_back(std::string(dirp->d_name));
    }
    closedir(dp);
    return 0;
}

void quit_handler(int signal=0) {
	v::term::kbmode(false);
	if (fd_data > 0) fclose(fd_data);
	if (fdev > 0) icm20602_close(fdev);
}

int main(int argc, const char* argv[]) {
	if (!v::filesystem::exdir("log")) v::filesystem::mkdir("log");
	FLAGS_alsologtostderr = true;
	google::SetLogDestination(google::GLOG_INFO, "./log/");
	google::InitGoogleLogging(argv[0]);

	if (!v::filesystem::exdir(DATA_DIR)) v::filesystem::mkdir(DATA_DIR);

	fdev = icm20602_open("/dev/spidev1.0");
	if (fdev < 0) { LOG(WARNING) << "cannot open icm20602 (spidev1.0)"; return -1; }
	icm20602_init(fdev);

	signal(SIGINT, quit_handler); // register quit_handler to return normal terminal mode when ctrl+c is pressed.

	char fnCalib[128] = {0};
	char fnCalibAcc[256] = {0}, fnCalibGyr[256] = {0}, fnCalibDat[256] = {0};
	std::unique_ptr<v::chrono::SyncTimer> sync = v::chrono::createSyncTimer();
	sync->arm(0.00025); // 250 us, 0.25 ms
	bool bExit = false;
	enum ModeCalib { WAIT = 0, GRAB, SEL, RUN, EXIT };
	int mode = 0; // 0: wait, 1: data achive, 2: running
	IMUData_t imu = {0};
	float initStaticTime = 25.f;
	int maxNumRotations = 50;
	while(true) {
		if (mode == WAIT) {
			printf("Type 'g' or 'G' to start gathering imu data\r\n");
			printf("Type 's' or 'S' to select imu dataset\r\n");
			printf("Type 'r' or 'R' to run calibration\r\n");
			printf("Type 'q' or 'Q' to exit program\r\n");
			bool bModeChange = false;
			v::term::kbmode(true);
			while (!bModeChange) {
				v::term::kbhit();
				int kb = v::term::getch();
				if (kb >= 0) {
					char key = (char)kb;
					switch (key) {
						case 'g': case 'G': mode = GRAB; bModeChange = true; break;
						case 's': case 'S': mode = SEL; bModeChange = true; break;
						case 'r': case 'R': mode = RUN; bModeChange = true; break;
						case 'q': case 'Q': mode = EXIT; bModeChange = true; break;
					}
				}
			}
			v::term::kbmode(false);
		}
		if (mode == GRAB) {
//			printf("Type initial static time (10-50): ");
//			scanf("%f", &initStaticTime);
//			printf("Type maximum rotations (30-150): ");
//			scanf("%d", &maxNumRotations);			
			printf("Type prefix of stored dataset (current: %s): ", fnCalib);
			scanf("%s", fnCalib);
			sprintf(fnCalibDat, "%s.dat", fnCalib);
			// data grab
			char fullpath[512];
			sprintf(fullpath, "%s/%s", DATA_DIR.c_str(), fnCalibDat);
			fd_data = fopen(fullpath, "w");
			printf("%s is created.\r\n", fullpath);
			CHECK(fd_data);
			// wait start
			int t_remain = 10;
			printf("Dont move. After %f secs, grabbing imu data will be start!\r\n", initStaticTime);
			uint64_t t0 = v::chrono::tic();
			double data_time0;
			int nimus = 0;
			while (v::chrono::toc(t0) < (initStaticTime+3)*1000*1000) {
				if (icm20602_read(fdev, &imu)) {	
					if (nimus == 0)  data_time0 = (double)imu.ts.tv_sec + imu.ts.tv_nsec/1e9;
					double data_time = (double)imu.ts.tv_sec + imu.ts.tv_nsec/1e9 - data_time0;
					fprintf(fd_data, "%lf, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", data_time,
						imu.gyro[0], imu.gyro[1], imu.gyro[2], imu.accl[0], imu.accl[1], imu.accl[2], 0, 0, 0, 0);
					nimus++;
				}
				sync->synchronize();
			}
			printf("Start to move (at least 20 movements are necessary)!\r\nType any key to finish grabbing imu data.\r\n");
			v::term::kbmode(true);
			bool bfin = false;
			nimus = 0;
			const int n10s = 200 * 10; // 200 Hz x 10 secs
			while (!bfin) {
				if (icm20602_read(fdev, &imu)) {	
					double data_time = (double)imu.ts.tv_sec + imu.ts.tv_nsec/1e9 - data_time0;
					if (nimus % n10s == 0) 	printf("%d: move\r\n", nimus / n10s);
					if (nimus / n10s == 20) printf("Proper number of imu data was collected. More data, better accuracy is achieved. Gan bat-tte!\r\n");
					if (nimus / n10s == 40) printf("Enough number of imu data was collected. Thing Ho-A~\r\n");
					if (nimus / n10s == 60) printf("Best performance will be reached. Now you can end this.\r\n");
					fprintf(fd_data, "%lf, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", data_time,
						imu.gyro[0], imu.gyro[1], imu.gyro[2], imu.accl[0], imu.accl[1], imu.accl[2], 0, 0, 0, 0);
					nimus++;
				}	
				if (v::term::kbhit(1) && v::term::getch() > 0) bfin = true;
			}
			fclose(fd_data); fd_data = NULL;
			v::term::kbmode(false);
			printf("%s is stored\r\n", fullpath);
			mode = WAIT;
		}
		if (mode == SEL) {
			std::vector<std::string> files;
			getdir(DATA_DIR,files);
			printf("In %s directory,\r\n", DATA_DIR.c_str());
			for (unsigned int i = 0;i < files.size(); i++) std::cout << files[i] << std::endl;
			printf("Type imu data without extension: ");
			scanf("%s", fnCalib);
			printf("%s is selected\r\n", fnCalib);
			mode = WAIT;
		}
		if (mode == RUN) {
			sprintf(fnCalibAcc, "%s.acc", fnCalib);
			sprintf(fnCalibGyr, "%s.gyr", fnCalib);
			printf("Type imu data without extension: ");
			scanf("%s", fnCalib);
			sprintf(fnCalibDat, "%s.dat", fnCalib);
			char fullpath[512];
			sprintf(fullpath, "%s/%s", DATA_DIR.c_str(), fnCalibDat);
			fd_data = fopen(fullpath, "r");
			if (!fd_data) { printf("Warning: %s doesn't exist. Return to main page\r\n", fnCalibDat); mode=WAIT;continue;}
			fclose(fd_data); fd_data = NULL;
			v::lidaralgo::CalibAcclGyro calibAcclGyro(fnCalibAcc,	fnCalibGyr, fullpath, 
				0, 1000000 /*inf*/, initStaticTime, maxNumRotations);
			calibAcclGyro.run(true);
			printf("Calibration is complete.\r\n");
			mode = WAIT;
		}
		if (mode == EXIT) break;
	}

	quit_handler();
}


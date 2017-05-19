#include <stdio.h>
#include <unistd.h> // for sleep
#include <string>
#include <iostream>

#include "processThread.h"
#include "ARobot.h"
#include "Thread.h"
#include "IMUData.h"
#include "RangeData.h"
#include "SerialPort.h"
#include "UartRx.h"
#include "UartTx.h"
#include <vector>

class NavThread : public Thread {
 public:
    NavThread(ARobot *robot)
        :myRobot(robot)
    {cnt = 0; bot_waypts = 0; first_iter = true;}

    virtual void run(void);
    static void readConfig(const char* filename, ARobot *robot);
	static void readCurrentMap(const char* filedir, const char* xmlname, ARobot *robot, Navigate2D &nav_rt);
	static void writeCurrentMap(const char* filedir, const char* xmlname, ARobot *robot, Navigate2D &nav_rt);
	static void Navigate(const char* filename, const char* xmlname, ARobot *robot, Navigate2D &nav_rt);
	static int WayPointNav(ARobot *robot, Navigate2D &nav_rt);
	int cnt;
	size_t bot_waypts;
	bool first_iter;
 protected:
    SerialPort *mPort;
    ARobot *myRobot;
 private:
 	char buf[64];
};
#endif // _PROCESS_THREAD_h_

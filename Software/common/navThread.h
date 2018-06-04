#ifndef _NAV_THREAD_h_
#define _NAV_THREAD_h_

#include <stdio.h>
#include <unistd.h> // for sleep
#include <string>
#include <iostream>

#include <time.h>
#include <vector>
#include "ARobot.h"
#include "IMUData.h"
#include "processThread.h"
#include "RangeData.h"
#include "SerialPort.h"
#include "Thread.h"
#include "UartRx.h"
#include "UartTx.h"

class NavThread : public Thread {
public:
    NavThread(ARobot *robot, bool isNew)
        : myRobot(robot), readMap(isNew)
    {
        cnt = 0;
        bot_waypts = 0;
        first_iter = true;
    }

    virtual void run(void);
    virtual void DestroyThread();

    void readConfig(const char* filename, ARobot *robot);
    void startNewMap(ARobot *robot, Navigate2D &nav_rt);
    void readCurrentMap(const char* filedir, const char* xmlname, ARobot *robot, Navigate2D &nav_rt);
    void writeCurrentMap(const char* filedir, const char* xmlname, ARobot *robot, Navigate2D &nav_rt);
    void Navigate(const char* filename, const char* xmlname, ARobot *robot, Navigate2D &nav_rt);
    int WayPointNav(ARobot *robot, Navigate2D &nav_rt);
    int cnt;
    size_t bot_waypts;
    bool first_iter;
    bool readMap;
protected:
    SerialPort *mPort;
    ARobot *myRobot;
private:
    Navigate2D nav; //main map class obj
    long int start_time;
    long int time_difference;
    struct timespec gettime_now;

#ifdef WIN32
    const char* fileConfig = "C:/projects/StormingRobots2017/Data/Mem/config.txt";
    const char* in_dir = "C:/projects/StormingRobots2017/Data";
    const char* rt_logname = "realtime/rcj_log";
    const char* xml_name = "map_data/mazemap";
    const char* map_dir = "C:/projects/StormingRobots2017/Data/map_data";
    const char* map_name = "map.xml";
    const char* leftcapture_file = "C:/projects/StormingRobots2017/rcj-code/Software/letter/randomFolder/capL.jpg";
    const char* rightcapture_file = "C:/projects/StormingRobots2017/rcj-code/Software/letter/randomFolder/capR.jpg";
#else
    const char* fileConfig = "/home/alex/projects/rcj-code/Software/common/Mem/config.txt";
    const char* in_dir = "/home/alex/projects/rcj-code/Data";
    const char* rt_logname = "realtime/rcj_log";
    const char* xml_name = "mazemap";
    const char* map_dir = "/home/alex/projects/rcj-code/Data/map_data";
    const char* map_name = "savemaze";
    const char* leftcapture_file = "/home/alex/projects/rcj-code/Software/letter/randomFolder/capL.jpg";
    const char* rightcapture_file = "/home/alex/projects/rcj-code/Software/letter/randomFolder/capR.jpg";
#endif
};
#endif // _NAV_THREAD_h_

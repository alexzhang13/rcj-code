#include <stdio.h>
#include <unistd.h> // for sleep
#include <string>
#include <iostream>

#include "processThread.h"
#include "navThread.h"
#include "ARobot.h"
#include "Thread.h"
#include "IMUData.h"
#include "RangeData.h"
#include "SerialPort.h"
#include "UartRx.h"
#include "UartTx.h"
#include <vector>

using namespace std;

int main(int argc,char **argv){
#ifdef WIN32
    const char* fileConfig = "C:/projects/StormingRobots2017/Data/Mem/config_test.txt";
    const char* in_dir = "C:/projects/StormingRobots2017/Data";
    const char* rt_logname = "realtime/rcj_log";
    const char* xml_name = "map_data/mazemap";
#else
    const char* fileConfig = "/home/alex/projects/rcj-code/Software/common/Mem/config_test.txt";
    const char* in_dir = "/home/alex/projects/rcj-code/Data";
    const char* rt_logname = "realtime/rcj_log";
    const char* xml_name = "map_data/mazemap";
#endif

    SerialPort *port = new SerialPort("/dev/ttyAMA0",115200);
	if(port == NULL)
		printf(" Serial port open failed\n");
	printf(".Start robot navigation\n");
    ARobot *myRobot = new ARobot(port);
    printf("Fault 1 Passed\n");
    //UartRx *uartrx = new UartRx(port, myRobot);
	//uartrx->setLogFile(in_dir, rt_logname);
    printf("Fault 2 Passed\n");
    //Process_T *process_thread = new Process_T(port, myRobot);
    printf("Fault 3 Passed\n");
    NavThread *nav_thread = new NavThread(myRobot);

    while(1) {
        sleep(0.01);
    }

    return 0;
}


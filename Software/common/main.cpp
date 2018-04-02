#include <stdio.h>
#include <unistd.h> // for sleep
#include <string>
#include <iostream>
#include <wiringPi.h>

#include <vector>
#include "../_headers/ARobot.h"
#include "../_headers/IMUData.h"
#include "../_headers/navThread.h"
#include "../_headers/dataThread.h"
#include "../_headers/testThread.h"
#include "../_headers/processThread.h"
#include "../_headers/RangeData.h"
#include "../_headers/SerialPort.h"
#include "../_headers/Thread.h"
#include "../_headers/UartRx.h"
#include "../_headers/UartTx.h"

using namespace std;

Thread *sThread; //spawned thread
void respawnThread(Thread *currThread);

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
    Thread *currThread;
    printf("Fault 1 Passed\n");
    UartRx *uartrx = new UartRx(port, myRobot);
	uartrx->setLogFile(in_dir, rt_logname);
    printf("Fault 2 Passed\n");
    Process_T *process_thread = new Process_T(port, myRobot);
    printf("Fault 3 Passed\n");

    wiringPiSetup();
    //spawnThread(sThread)

    NavThread *nav_thread = new NavThread(myRobot, false);
    //TestThread *test_thread = new TestThread(myRobot);
    //DataThread *data_thread = new DataThread(myRobot);

    while(1) {
    	//if(Button Pressed) when UnPressed, call spawnThread
        sleep(0.01);
    }

    return 0;
}

/**
 * 0 0 --> Restart Navigation Program
 * 0 1 --> Use previous data (Silver)
 * 1 0 --> Collect Data
 * 1 1 --> Tester Thread (i.e. Testing kNN PiCam, etc.)
 */
void respawnThread(Thread *currThread) {
	/*TODO: Pause current thread if it's paused? (check this)
	//getPin 22 23 24 27
	int currChoice = getPin(23) + getPin(24)*2;
	switch(currChoice) {
		case 0: //0 0
			break;
		case 1: //1 0
			break;
		case 2: //0 1
			break;
		case 3: //1 1
			break;
	}
	*/
}




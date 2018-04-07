#include <stdio.h>
#include <unistd.h> // for sleep
#include <string>
#include <iostream>
#include <wiringPi.h>

#include <vector>
#include "ARobot.h"
#include "IMUData.h"
#include "navThread.h"
#include "dataThread.h"
#include "testThread.h"
#include "processThread.h"
#include "RangeData.h"
#include "SerialPort.h"
#include "Thread.h"
#include "UartRx.h"
#include "UartTx.h"

using namespace std;

void spawnThread(Thread *currThread,  ARobot *myRobot);
void stopThread(Thread *currThread);

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
    bool isRunning = false; //start program button
    Thread *sThread; //spawned thread

    //Set up Wires
    wiringPiSetup();
    pinMode(22, OUTPUT); //LED Pin
    pinMode(23, INPUT); //Dipswitch PIN 2
    pinMode(24, INPUT); //Dipswitch PIN 1
    pinMode(27, INPUT); //Push Pin (Toggle Program)
    printf("WiringPI Init Passed");

    SerialPort *port = new SerialPort("/dev/ttyAMA0",115200);
    if(port == NULL)
          printf(" Serial port open failed\n");
    printf(".Start robot navigation\n");
    ARobot *myRobot = new ARobot(port);
    printf("ARobot Init Passed\n");

    UartRx *uartrx = new UartRx(port, myRobot);
    uartrx->setLogFile(in_dir, rt_logname);
    printf("UartRx Thread Init Passed\n");

    Process_T *process_thread = new Process_T(port, myRobot);
    printf("Process Thread Init Passed\n");

    while(1) {
        //printf("DS 1: %d\tDS 2: %d\tPP: %d\n", digitalRead(24), digitalRead(23), digitalRead(27));

        if(digitalRead(27)==0 && !isRunning) { //button is pressed when off
            spawnThread(currThread);
            isRunning = true;
        } else if(digitalRead(x)==1 && isRunning) {
            stopThread(currThread);
            isRunning = false;
        }
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
void spawnThread(Thread *currThread, ARobot *myRobot) {
    //getPin 22 23 24 27
    int currChoice = digitalRead(24) + digitalRead(23)*2;
    switch(currChoice) {
        case 0: //0 0
            currThread = new NavThread(myRobot, false);
            break;
        case 1: //1 0
            currThread = new NavThread(myRobot, true);
            break;
        case 2: //0 1
            currThread = new DataThread(myRobot);
            break;
        case 3: //1 1
            currThread = new TestThread(myRobot);
            break;
	}
}

void stopThread(Thread *currThread) {
    delete currThread;
}




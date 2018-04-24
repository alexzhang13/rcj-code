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

SerialPort *port;
ARobot *myRobot;
UartRx *uartrx;
Process_T *process_thread;

void spawnThread(Thread *currThread,  ARobot *myRobot);
void stopThread(Thread *currThread, ARobot *myRobot);

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
    bool reset = false; //flag
    int iteration = 0;
    Thread *currThread; //spawned thread

    //Set up Wires, Below is the wiringPi -> Pi Rev.3 GPIO Mapping
    //22 --> 3 (WiringPI)
    //23 --> 4
    //24 --> 5 [WORKS]
    //27 --> 2 [WORKS]

    wiringPiSetup();
    pinMode(3, OUTPUT); //LED Pin
    pinMode(4, INPUT); //Dipswitch PIN 2
    pinMode(5, INPUT); //Dipswitch PIN 1
    pinMode(2, INPUT); //Push Pin (Toggle Program)
    printf("WiringPI Init Passed");
#if 0
    port = new SerialPort("/dev/ttyAMA0",115200);
    if(port == NULL)
        printf("Serial port open failed\n");
    printf("Start robot navigation...\n");
    myRobot = new ARobot(port);
    printf("ARobot Init Passed...\n");

    uartrx = new UartRx(port, myRobot);
    uartrx->setLogFile(in_dir, rt_logname);
    printf("UartRx Thread Init Passed\n");

    process_thread = new Process_T(port, myRobot);
    printf("Process Thread Init Passed\n");
#endif

    while(1) {
        if(iteration % 1000 == 0) {
            if(digitalRead(2)==0 && !isRunning && reset) { //button is pressed when off
                port = new SerialPort("/dev/ttyAMA0",115200);
                if(port == NULL)
                    printf("Serial port open failed\n");
                printf("Start robot navigation...\n");
                myRobot = new ARobot(port);
                printf("ARobot Init Passed...\n");

                uartrx = new UartRx(port, myRobot);
                uartrx->setLogFile(in_dir, rt_logname);
                printf("UartRx Thread Init Passed\n");

                process_thread = new Process_T(port, myRobot);
                printf("Process Thread Init Passed\n");
                sleep(3);

                printf("Spawning New Thread...\n");
                myRobot->Reset();
                spawnThread(currThread, myRobot);
                isRunning = true;
                reset = false;
            } else if(digitalRead(2)==0 && isRunning && reset) {
                printf("Thread Killed...\n");
                stopThread(currThread, myRobot);
                sleep(0.1);
                delete process_thread;
                sleep(0.1);
                delete uartrx;
                sleep(0.1);
                delete myRobot;
                sleep(0.1);
                delete port;
                sleep(0.1);
                isRunning = false;
                reset = false;
            } else if (digitalRead(2)==1) {
                reset = true;
            }
        }
        ++iteration;
        sleep(0.01);
    }

    return 0;
}

/**
 * 0 = On, 1 = Off
 * 0 0 --> Restart Navigation Program
 * 0 1 --> Use previous data (Silver)
 * 1 0 --> Collect Data
 * 1 1 --> Tester Thread (i.e. Testing kNN PiCam, etc.)
 */
void spawnThread(Thread *currThread, ARobot *myRobot) {
    int currChoice = digitalRead(5) + digitalRead(4)*2;

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

void stopThread(Thread *currThread, ARobot *myRobot) {
    delete currThread;
    myRobot->picam.close();
    myRobot->Reset();
}





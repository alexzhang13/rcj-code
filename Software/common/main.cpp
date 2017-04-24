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

using namespace std;

void readConfig(const char* filename, ARobot *robot);

int main(int argc,char **argv){
    const char* fileConfig = "./Mem/config.txt";

    SerialPort *port = new SerialPort("/dev/ttyAMA0",115200);
    ARobot *myRobot = new ARobot(port);
    UartRx *uartrx = new UartRx(port, myRobot);
    //UartTx *uarttx = new UartTx(port);
    Process_T *process_thread = new Process_T(port, myRobot);

    readConfig(fileConfig, myRobot);

    //readCurrentMap(unsigned char* filename);

    //Nagivate(robot);

    //Motion(robot);   

    while(1) sleep(1); //make sure main never returns

    return 0;
}

void readConfig(const char* filename, ARobot *robot)
{
    FILE *datafile;
    if (filename == NULL)
        return;

    datafile = fopen(filename, "r");
    fscanf(datafile, "%d %d %d %d %d", &robot->black_thresh, &robot->silver_thresh, &robot->white_thresh, &robot->threshLeft, &robot->threshRight);
}

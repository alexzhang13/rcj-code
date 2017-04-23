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

int main(int argc,char **argv){
    SerialPort *port = new SerialPort("/dev/ttyAMA0",115200);
    ARobot *robot = new ARobot(port);
    UartRx *uartrx = new UartRx(port, robot);
    //UartTx *uarttx = new UartTx(port);
    Process_T *process_thread = new Process_T(port, robot);

    readConfig(unsigned char* filename);

    //readCurrentMap(unsigned char* filename);

    //Nagivate(robot);

    //Motion(robot);   

    while(1) sleep(1); //make sure main never returns

    return 0;
}

int readConfig(unsigned char* filename)
{
    FILE *datafile;
    if (filename == NULL)
        return -1;

    datafile = fopen(filename, "r");
    fscanf(datafile, "%d %d %d %d %d", robot->black_thresh, robot->silver_thresh, robot->white_thresh, robot->threshLeft, robot->threshRight);
}

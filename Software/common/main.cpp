#include <stdio.h>
#include <unistd.h> // for sleep

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
    Process_T *process_thread = new Process_T(port, robot);

    //UartTx *uarttx = new UartTx(port);

    //readConfig(unsigned char* filename);

    //readCurrentMap(unsigned char* filename);

    //Nagivate(robot);

    //Motion(robot);   

    while(1) sleep(1); //make sure main never returns

    return 0;
}


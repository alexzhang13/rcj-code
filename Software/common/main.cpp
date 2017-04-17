#include <stdio.h>
#include <unistd.h> // for sleep

#include "IMUData.h"
#include "RangeData.h"
#include "SerialPort.h"
#include "UartRx.h"
#include "UartTx.h"
#include <vector>

std::vector<IMUData> imuDataList;
std::vector<RangeData> rangeDataList;

int main(int argc,char **argv){
#if 1
    char buf[64];
    //SerialPort *port = new SerialPort("/dev/ttyUSB0",115200);
    SerialPort *port = new SerialPort("/dev/ttyAMA0",115200);

    // loopback
    //port->printf("hello world!\n");
	while(1) {
		for(int i = 0; i < 64; i++) {
			buf[i] = ' ';
		}
    	port->fgets(buf,64);
        for(int i = 0; i < 64; i++) {
            if(buf[i] == 'i') { //check if it's IMU
                curr_imu.parseData(buf);
                curr_imu.runFilter();
                break;
            } else if (buf[i] == 'r') {

            }
        }

		//printf("%s\n",buf);
	}
#endif

    //UartRx *uartrx = new UartRx(port);
    //UartTx *uarttx = new UartTx(port);
    //UartRx *uartrx = new UartRx(0);
    //UartTx *uarttx = new UartTx(0);

    //while(1) sleep(100);
}
void parseIMU() {

}

void parseRange() {

}

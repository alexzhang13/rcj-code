#include "UartRx.h"
#include <unistd.h> // for sleep

void UartRx::run(void){     
    while(1) {
    	for(int i = 0; i < 64; i++) {
    	    buf[i] = ' ';
   		}
    	mPort->fgets(buf,64);
    	for(int i = 0; i < 64; i++) {
        	if(buf[i] == 'i') { //check if it's IMU
            	parseIMU(buf);
            	break;
        	} else if (buf[i] == 'r') {
             	parseRange(buf);
             	break;
         	} else if (buf[i] == 't') {
             	parseTemp(buf);
              	break;
        	} else if (buf[i] == 'l') {
             	parseLight(buf);
             	break;
        	} else if (buf[i] == 'm') {
              	myRobot->currState = IDLE;
             	break;
            } else if (buf[i] == 'l') {
              	myRobot->currState = IDLE;
             	break;
            } else if (buf[i] == 'd') {
              	myRobot->currState = IDLE;
             	break;
        	} else {
            	printf("Error in parsing");
           	 	break;
        	}
    	}
        	sleep(1);
    }
}

void UartRX::storeIMU(char* buf) {
    IMUData curr_imu;
    curr_imu.storeCommand(buf);
    myRobot->imuParseList.push(curr_imu); //push imu data
}

void UartRX::storeRange(char* buf) {
    RangeData curr_range;
    curr_range.storeCommand(buf);
    myRobot->rangeParseList.push(curr_range); //push range data
}

void UartRX::storeTemp(char* buf) {
    TempData curr_temp;
    curr_temp.storeCommand(buf);
    myRobot->tempParseList.push(curr_temp); //push temp data
}

void UartRX::storeLight(char* buf) {
    LightData curr_light;
    curr_light.storeCommand(buf);
    myRobot->lightParseList.push(curr_light); //push light data
}
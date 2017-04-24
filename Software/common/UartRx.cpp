#include "UartRx.h"
#include <unistd.h> // for sleep
#include "IMUData.h"
#include "TempData.h"
#include "LightData.h"
#include "RangeData.h"

void UartRx::run(void){     
    while(1) {
    	for(int i = 0; i < 64; i++) {
    	    buf[i] = ' ';
   		}
    	mPort->fgets(buf,64);
    	for(int i = 0; i < 64; i++) {
        	if(buf[i] == 'i') { //check if it's IMU
            	storeIMU(buf, myRobot);
            	break;
        	} else if (buf[i] == 'r') {
             	storeRange(buf, myRobot);
             	break;
         	} else if (buf[i] == 't') {
             	storeTemp(buf, myRobot);
              	break;
        	} else if (buf[i] == 'l') {
             	storeLight(buf, myRobot);
             	break;
        	} else if (buf[i] == 'm') {
              	myRobot->currState = IDLE; //IDLE
             	break;
            } else if (buf[i] == 'l') {
              	myRobot->currState = IDLE; //IDLE
             	break;
            } else if (buf[i] == 'd') {
              	myRobot->currState = IDLE; //IDLE
             	break;
        	} else {
            	printf("Error in parsing");
           	 	break;
        	}
    	}
        	sleep(1);
    }
}

void UartRX::storeIMU(char* buf, ARobot *mRobot) {
    IMUData curr_imu;
    curr_imu.storeCommand(buf);
    mRobot->imuParseList.push(curr_imu); //push imu data
}

void UartRX::storeRange(char* buf, ARobot *mRobot) {
    RangeData curr_range;
    curr_range.storeCommand(buf);
    mRobot->rangeParseList.push(curr_range); //push range data
}

void UartRX::storeTemp(char* buf, ARobot *mRobot) {
    TempData curr_temp;
    curr_temp.storeCommand(buf);
    mRobot->tempParseList.push(curr_temp); //push temp data
}

void UartRX::storeLight(char* buf, ARobot *mRobot) {
    LightData curr_light;
    curr_light.storeCommand(buf);
    mRobot->lightParseList.push(curr_light); //push light data
}
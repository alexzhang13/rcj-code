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
            	storeIMU(buf);
            	break;
        	} else if (buf[i] == 'r') {
             	storeRange(buf);
             	break;
         	} else if (buf[i] == 't') {
             	storeTemp(buf);
              	break;
        	} else if (buf[i] == 'l') {
             	storeLight(buf);
             	break;
        	} else if (buf[i] == 'm') {
              	myRobot->currState = ARobot::IDLE; //IDLE
             	break;
            } else if (buf[i] == 'l') {
              	myRobot->currState = ARobot::IDLE; //IDLE
             	break;
            } else if (buf[i] == 'd') {
              	myRobot->currState = ARobot::IDLE; //IDLE
             	break;
        	} else {
            	printf("Error in parsing");
            	printf(buf[i]);
           	 	break;
        	}
    	}
        	sleep(1);
    }
}

void UartRx::storeIMU(char* buf) {
    IMUData curr_imu;
    curr_imu.storeCommand(buf);
    myRobot->imuParseList.push(curr_imu); //push imu data
}

void UartRx::storeRange(char* buf) {
    RangeData curr_range(myRobot);
    curr_range.storeCommand(buf);
    myRobot->rangeParseList.push(curr_range); //push range data
}

void UartRx::storeTemp(char* buf) {
    TempData curr_temp;
    curr_temp.storeCommand(buf, myRobot->threshLeft, myRobot->threshRight);
    myRobot->tempParseList.push(curr_temp); //push temp data
}

void UartRx::storeLight(char* buf) {
    LightData curr_light;
    curr_light.storeCommand(buf, myRobot->black_thresh, myRobot->silver_thresh, myRobot->white_thresh);
    myRobot->lightParseList.push(curr_light); //push light data
}

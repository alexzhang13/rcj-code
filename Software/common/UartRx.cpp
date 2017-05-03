#include "UartRx.h"
#include <unistd.h> // for sleep
#include "IMUData.h"
#include "TempData.h"
#include "LightData.h"
#include "RangeData.h"
#include <stdio.h>
#include <string.h>

void UartRx::run(void){   
	int32_t ts;
	char c;
    while(1) {
		memset(mBuf,'\0', 128);
    	char *retval = mPort->fgets(mBuf,64);
		if( retval == NULL) {
			sleep(0.001);
			continue;
        }
	
		sscanf(mBuf, "%i %c", &ts, &c); 
        if(c == 'i') { //check if it's IMU
            storeIMU(mBuf);
        } else if (c == 'r') {
            storeRange(mBuf);
        } else if (c == 't') {
            storeTemp(mBuf);
       	} else if (c == 'l') {
            storeLight(mBuf);
       	} else if (c == 'm') {
            myRobot->currState = ARobot::IDLE; //IDLE
        } else if (c == 'l') {
            myRobot->currState = ARobot::IDLE; //IDLE
        } else if (c == 'd') {
            myRobot->currState = ARobot::IDLE; //IDLE
       	} else {}
    }
	return;
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

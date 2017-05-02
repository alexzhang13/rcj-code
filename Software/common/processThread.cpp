#include "processThread.h"
#include <unistd.h> // for sleep

void Process_T::run(void){  
	bool processed_status = false;   
    while(1) {

        while(myRobot->imuParseList.size() > 0) {
		   printf("imu parse list size = %d\n", myRobot->imuParseList.size());
           printf("imu data list size = %d\n", myRobot->imuDataList.size());
    	   myRobot->ParseIMU();
           myRobot->ClearIMU();
		   processed_status = true;
        }

        while(myRobot->rangeParseList.size() > 0) {
           myRobot->ParseRange();
           myRobot->ClearRange();
		   processed_status = true;
        }

        while(myRobot->tempParseList.size() > 0) {
           myRobot->ParseTemp();
           myRobot->ClearTemp();
		   processed_status = true;
        }
        while(myRobot->lightParseList.size() > 0) {
           myRobot->ParseLight();
           processed_status = true;
           //Inside CheckTileLight() there is a Clear Light-esque function so it's not necessary here
        }

        if(!processed_status)
			sleep(0.01);
    }
	return;
}

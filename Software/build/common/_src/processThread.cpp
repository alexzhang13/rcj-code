#include "../_headers/processThread.h"

#include <unistd.h> // for sleep

void Process_T::run(void){  
	bool processed_status;   

    while(1) {
		    processed_status = false; 
        while(myRobot->imuParseList.size() > 0) {
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
           myRobot->ClearLight();
           processed_status = true;
           //Inside CheckTileLight() there is a Clear Light-esque function so it's not necessary here
        }
        while(myRobot->scanParseList.size() > 0) {
          myRobot->ParseScan();
          processed_status = true;
        }
        if(!processed_status)
			    sleep(0.01);
    }
	return;
}

#include "processThread.h"
#include <unistd.h> // for sleep

void Process_T::run(void){     
    while(1) {
        while(myRobot->imuParseList.size() > 0) {
    	   myRobot->ParseIMU();
           myRobot->ClearIMU();
        }
        while(myRobot->rangeParseList.size() > 0) {
           myRobot->ParseRange();
           myRobot->ClearRange();
        }
        while(myRobot->tempParseList.size() > 0) {
           myRobot->ParseTemp();
           myRobot->ClearTemp();
        }
        while(myRobot->lightParseList.size() > 0) {
           myRobot->ParseLight();
           //Inside CheckTileLight() there is a Clear Light-esque function so it's not necessary here
        }
        

        sleep(1);
    }
}

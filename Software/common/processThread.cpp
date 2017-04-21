#include "processThread.h"
#include <unistd.h> // for sleep

void Process_T::run(void){     
    while(1) {
        while(myRobot->imuParseList.size() > 0) {
    	   myRobot->ParseIMU();
        }
        if(myRobot->rangeParseList.size() > 0) {
           myRobot->ParseRange();
        }
        if(myRobot->tempParseList.size() > 0) {
           myRobot->ParseTemp();
        }
        if(myRobot->lightParseList.size() > 0) {
           myRobot->ParseLight();
        }
        myRobot->ClearVectors();

        sleep(1);
    }
}

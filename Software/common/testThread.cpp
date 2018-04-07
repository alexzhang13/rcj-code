#include "testThread.h"

using namespace std;

void TestThread::run(void){
    sleep(1);
    while(1) {
        myRobot->TestRangeSensors();
        sleep(0.5);
    }
}

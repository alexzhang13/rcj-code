#include "testThread.h"

using namespace std;

void TestThread::run(void){
    printf("Test Thread Started...\n");
    sleep(1);
    while(1) {
        myRobot->TestRangeSensors();
        sleep(2);
    }
    return;
}

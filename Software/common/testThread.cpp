#include "testThread.h"

using namespace std;

void TestThread::run(void){
    printf("Test Thread Started...\n");
#if 0
    myRobot->picam.cameraOpen(320, 240); //start up camera
    printf("Capture 1\n");
    myRobot->picam.frameCapture();
    sleep(0.5);
    printf("Capture 2\n");
    myRobot->picam.frameCapture();
    sleep(0.5);
    printf("Capture 3\n");
    myRobot->picam.frameCapture();
    sleep(0.5);

    myRobot->CheckVictimVisual();
    printf("Pushed Mat's into ImgList<Mat>\n");
    sleep(0.5);
    printf("Side of Victim: %d\n", myRobot->ProcessImage_Victim());
    sleep(0.5);
    myRobot->picam.close();
#endif
    sleep(1);
    while(1) {
        printf("Testing...");
        sleep(5);
    }
    return;
}

#include "testThread.h"

using namespace std;

void TestThread::run(void){
    printf("Test Thread Started...\n");

    myRobot->picam.cameraOpen(720, 480); //start up camera
    sleep(1);
    myRobot->picam.frameCapture("/home/alex/projects/rcj-code/Software/letter/randomFolder/img.jpg");
    sleep(1);
    printf("Capture 1\n");

    FILE * f = popen( "python /home/alex/projects/rcj-code/Software/letter/identify.py /home/alex/projects/rcj-code/Software/letter/randomFolder/img.jpg", "r" );
    char buf[ 150 ];
    fgets(buf, 150, f);
    fprintf( stdout, "%s", buf  );
    pclose( f );
    myRobot->picam.close();

    sleep(1);
    while(1) {
        printf("%s\n", buf);
        sleep(5);
    }
    return;
}

#include "testThread.h"

using namespace std;

void TestThread::run(void){
    printf("Test Thread Started...\n");
    myRobot->picam.cameraOpen(720, 480);
    sleep(3);
    myRobot->picam.frameCapture("/home/alex/projects/rcj-code/Software/letter/randomFolder/img.jpg");
    sleep(1);
    myRobot->picam.close();
    printf("Capture 1\n");

    FILE * f = popen( "python /home/alex/projects/rcj-code/Software/letter/identify.py /home/alex/projects/rcj-code/Software/letter/randomFolder/img.jpg", "r" );
    char* buf;
    fgets(buf, 10, f);
    //fprintf( stdout, "%c", buf[0]);
    pclose( f );
    sleep(1);

    while(1) {
        printf("%c\n", buf[0]);
        sleep(8);
    }
    return;
}

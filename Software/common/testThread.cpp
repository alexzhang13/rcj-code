#include "testThread.h"

using namespace std;

void TestThread::run(void){
    printf("Test Thread Started...\n");
    sleep(3);
    myRobot->picam.frameCapture("/home/alex/projects/rcj-code/Software/letter/randomFolder/img.jpg");
    sleep(1);
    printf("Capture 1\n");
    myRobot->picam.close();

    FILE * f = popen( "python /home/alex/projects/rcj-code/Software/letter/identify.py /home/alex/projects/rcj-code/Software/letter/randomFolder/img.jpg", "r" );
    char* buf;
    fgets(buf, 1, f);
    fprintf( stdout, "%c", buf[0]);
    pclose( f );
    sleep(1);

    while(1) {
        printf("%c\n", buf[0]);
        sleep(8);
    }
    return;
}

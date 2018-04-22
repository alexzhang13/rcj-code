#include "testThread.h"

using namespace std;

void TestThread::run(void){
    printf("Test Thread Started...\n");
    sleep(1);
    myRobot->picam.frameCapture("/home/alex/projects/rcj-code/Software/letter/randomFolder/img.jpg");
    sleep(1);
    printf("Capture 1\n");
    myRobot->picam.close();

    FILE * f = popen( "python /home/alex/projects/rcj-code/Software/letter/identify.py /home/alex/projects/rcj-code/Software/letter/randomFolder/img.jpg", "r" );
    char buf[ 10 ];
    fgets(buf, 10, f);
    fprintf( stdout, "%s", buf  );
    pclose( f );
    sleep(1);

    while(1) {
        printf("%s\n", buf);
        sleep(8);
    }
    return;
}

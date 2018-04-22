#include <stdio.h>

int main() {
    FILE * f = popen( "python3 identify.py randomFolder/letterPic0.jpg", "r" );
    char buf[ 10 ];
    fgets(buf, 10, f);
    fprintf( stdout, "%s", buf  );
    pclose( f );
}

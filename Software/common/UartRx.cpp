#include "UartRx.h"
#include <unistd.h> // for sleep

void UartRx::run(void){
    while(1){
        // char buf[32];
        // mPort->fgets(buf,32);
        sleep(1);
        printf("hello world Rx\n");
    }
}

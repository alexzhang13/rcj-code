#include "UartTx.h"
#include <unistd.h> // for sleep

void UartTx::run(void){
    while(1){
        // char buf[32];
        // mPort->fgets(buf,32);
        sleep(2);
        printf("hello world Tx\n");
    }
}

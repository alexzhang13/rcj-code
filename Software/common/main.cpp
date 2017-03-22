#include <stdio.h>
#include <unistd.h> // for sleep

#include "SerialPort.h"
#include "UartRx.h"
#include "UartTx.h"

int main(int argc,char **argv){
#if 0
    char buf[30];
    //SerialPort *port = new SerialPort("/dev/ttyUSB0",115200);
    SerialPort *port = new SerialPort("/dev/ttyAMA0",115200);

    // loopback
    port->printf("hello world!\n");
    port->fgets(buf,30);

    printf("%s\n",buf);
#endif

    //UartRx *uartrx = new UartRx(port);
    //UartTx *uarttx = new UartTx(port);
    UartRx *uartrx = new UartRx(0);
    UartTx *uarttx = new UartTx(0);

    while(1) sleep(100);
}

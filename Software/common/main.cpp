#include <stdio.h>
#include <unistd.h> // for sleep

#include "SerialPort.h"
#include "UartRx.h"
#include "UartTx.h"

int main(int argc,char **argv){
#if 1
    char* curr_line;
    //SerialPort *port = new SerialPort("/dev/ttyUSB0",115200);
    SerialPort *port = new SerialPort("/dev/ttyAMA0",115200);

    // loopback
    //port->printf("hello world!\n");
    port->fgetln(curr_line);

    printf("%s\n", curr_line);
#endif

    //UartRx *uartrx = new UartRx(port);
    //UartTx *uarttx = new UartTx(port);
    //UartRx *uartrx = new UartRx(0);
    //UartTx *uarttx = new UartTx(0);

    //while(1) sleep(100);
}

#include <stdio.h>
#include <unistd.h> // for sleep

#include "SerialPort.h"
#include "UartRx.h"
#include "UartTx.h"

int main(int argc,char **argv){
#if 1
    char buf[40];
    //SerialPort *port = new SerialPort("/dev/ttyUSB0",115200);
    SerialPort *port = new SerialPort("/dev/ttyAMA0",115200);

    // loopback
    //port->printf("hello world!\n");
	while(1) {
		for(int i = 0; i < 40; i++) {
			buf[i] = ' ';
		}
    	port->fgets(buf,40);
		printf("%s\n",buf);
	}
#endif

    //UartRx *uartrx = new UartRx(port);
    //UartTx *uarttx = new UartTx(port);
    //UartRx *uartrx = new UartRx(0);
    //UartTx *uarttx = new UartTx(0);

    //while(1) sleep(100);
}

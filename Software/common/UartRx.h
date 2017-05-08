#ifndef _UartRx_h_
#define _UartRx_h_

#include "ARobot.h"
#include "Thread.h"
#include "SerialPort.h"
#include "Thread.h"
#include "SerialPort.h"

class ARobot;
class UartRx : public Thread {
 public:
    UartRx(SerialPort *port, ARobot *robot)
        :mPort(port), myRobot(robot)
    {}

    void storeIMU(char* buf);
	void storeRange(char* buf);
	void storeTemp(char* buf);
	void storeLight(char* buf);
    virtual void run(void);
 protected:
    SerialPort *mPort;
    ARobot *myRobot;
 private:
 	char mBuf[128];
};
#endif // _UartRx_h_

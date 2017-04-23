#ifndef _UartRx_h_
#define _UartRx_h_

#include "ARobot.h"
#include "Thread.h"
#include "SerialPort.h"
#include "Thread.h"
#include "IMUData.h"
#include "RangeData.h"
#include "SerialPort.h"

class UartRx : public Thread {
 public:
    UartRx(SerialPort *port, ARobot *robot)
        :mPort(port), myRobot(robot)
    {}

    virtual void run(void);
 protected:
    SerialPort *mPort;
    ARobot *myRobot;
    void storeIMU(char* buf);
	void storeRange(char* buf);
	void storeTemp(char* buf);
	void storeLight(char* buf);
 private:
 	char buf[64];
};
#endif // _UartRx_h_

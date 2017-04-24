#ifndef _PROCESS_THREAD_h_
#define _PROCESS_THREAD_h_

#include "ARobot.h"
#include "Thread.h"
#include "SerialPort.h"
#include "Thread.h"
#include "IMUData.h"
#include "RangeData.h"
#include "SerialPort.h"

class Process_T : public Thread {
 public:
    Process_T(SerialPort *port, ARobot *robot)
        :mPort(port), myRobot(robot)
    {}

    virtual void run(void);
 protected:
    SerialPort *mPort;
    ARobot *myRobot;
 private:
 	char buf[64];
};
#endif // _PROCESS_THREAD_h_

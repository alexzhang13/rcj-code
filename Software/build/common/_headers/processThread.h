#ifndef _PROCESS_THREAD_h_
#define _PROCESS_THREAD_h_

#include "../_headers/ARobot.h"
#include "../_headers/IMUData.h"
#include "../_headers/RangeData.h"
#include "../_headers/SerialPort.h"
#include "../_headers/SerialPort.h"
#include "../_headers/Thread.h"
#include "../_headers/Thread.h"

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

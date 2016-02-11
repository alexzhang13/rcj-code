#ifndef COMMON__HEADERS_TESTTHREAD_H_
#define COMMON__HEADERS_TESTTHREAD_H_


#include <stdio.h>
#include <unistd.h> // for sleep
#include <string>
#include <iostream>

#include <vector>
#include "ARobot.h"
#include "IMUData.h"
#include "processThread.h"
#include "RangeData.h"
#include "SerialPort.h"
#include "Thread.h"
#include "UartRx.h"
#include "UartTx.h"

class TestThread : public Thread {
 public:
    TestThread(ARobot *robot) : myRobot(robot) {}
    virtual void run(void);

	bool readMap;
 protected:
    SerialPort *mPort;
    ARobot *myRobot;
 private:
 	#ifdef WIN32

	#else

	#endif
};


#endif /* COMMON__HEADERS_TESTTHREAD_H_ */

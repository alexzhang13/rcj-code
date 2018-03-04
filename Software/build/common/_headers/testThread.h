#ifndef COMMON__HEADERS_TESTTHREAD_H_
#define COMMON__HEADERS_TESTTHREAD_H_


#include <stdio.h>
#include <unistd.h> // for sleep
#include <string>
#include <iostream>

#include <vector>
#include "../_headers/ARobot.h"
#include "../_headers/IMUData.h"
#include "../_headers/processThread.h"
#include "../_headers/RangeData.h"
#include "../_headers/SerialPort.h"
#include "../_headers/Thread.h"
#include "../_headers/UartRx.h"
#include "../_headers/UartTx.h"

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

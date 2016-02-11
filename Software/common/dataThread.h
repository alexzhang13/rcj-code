#ifndef COMMON_DATATHREAD_H_
#define COMMON_DATATHREAD_H_

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

class DataThread : public Thread {
 public:
    DataThread(ARobot *robot) : myRobot(robot) {}
    virtual void run(void);

 protected:
    SerialPort *mPort;
    ARobot *myRobot;
 private:
 	#ifdef WIN32

	#else

	#endif
};



#endif /* COMMON_DATATHREAD_H_ */

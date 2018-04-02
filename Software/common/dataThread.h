#ifndef COMMON_DATATHREAD_H_
#define COMMON_DATATHREAD_H_

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

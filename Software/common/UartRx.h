#ifndef _UartRx_h_
#define _UartRx_h_

#include <string>
#include <stdio.h>
#include "ARobot.h"
#include "SerialPort.h"
#include "SerialPort.h"
#include "Thread.h"
#include "Thread.h"

class ARobot;
class UartRx : public Thread {
 public:
    UartRx(SerialPort *port, ARobot *robot);
	~UartRx();
	
    void storeIMU(char* buf);
	void storeRange(char* buf);
	void storeTemp(char* buf);
	void storeLight(char* buf);
    void storeScan(char* buf);
    virtual void run(void);
	
	bool setLogFile(const char* filedir, const char *filename);
 protected:
    SerialPort *mPort;
    ARobot *myRobot;
	bool runLogFile();
 private:
 	char mBuf[128];
	std::string mLogName;
	uint32_t mCnt;
	uint32_t mNum;
	FILE *mpFile;
};
#endif // _UartRx_h_

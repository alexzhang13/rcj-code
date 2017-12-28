#include "../_headers/UartRx.h"

#include <unistd.h> // for sleep
#include <stdio.h>
#include <string.h>
#include "../_headers/IMUData.h"
#include "../_headers/LightData.h"
#include "../_headers/RangeData.h"
#include "../_headers/TempData.h"

#ifdef WIN32
#include <windows.h>
#include <sys/utime.h>
#include <sstream>
#include <ctime>
#else
#include <ctime>
#endif

static std::string getCurDate()
{
	std::string cur_date;
#ifdef WIN32
	SYSTEMTIME  system_time;
	GetLocalTime(&system_time);
	cur_date = std::to_string(system_time.wYear) + "_" +
		std::to_string(system_time.wMonth) + "_" +
		std::to_string(system_time.wDay);
#else
	std::stringstream currentDateTime;
	// current date/time based on current system
	time_t ttNow = time(0);
	struct tm * now = localtime( & ttNow );
   cur_date = std::to_string(now->tm_year + 1900) + std::to_string(now->tm_mon + 1) + 		std::to_string(now->tm_mday);
#endif
	return cur_date;	
}

UartRx::UartRx(SerialPort *port, ARobot *robot) :mPort(port), myRobot(robot)
{ 
	mCnt = 0;
	mNum = 0;
	mLogName.clear();
	mpFile = NULL;
}
	
UartRx::~UartRx()
{
	mLogName.clear();
	if(mpFile)
		fclose(mpFile);
	mpFile = NULL;
}

void UartRx::run(void){   
	int32_t ts;
	char c, m;
    while(1) {
		memset(mBuf,'\0', 128);
    	char *retval = mPort->fgets(mBuf,64);
		if( retval == NULL) {
			sleep(0.001);
			continue;
        }
		runLogFile();
		sscanf(mBuf, "%i %c", &ts, &c); 
        if(c == 'i') { //check if it's IMU
            storeIMU(mBuf);
        } else if (c == 'r') {
            storeRange(mBuf);
        } else if (c == 't') {
            //storeTemp(mBuf);
       	} else if (c == 'l') {
            //storeLight(mBuf);
        } else if (c == 'y') {
        	storeScan(mBuf);
        } else if (c == 'z') {
        	//myRobot->picam.frameCapture();
        	printf("Data Sent!\n");
       	} else if (c == 'm') {
            myRobot->currState = ARobot::DATA; //IDLE
            printf("motor is finished\n");
            if(myRobot->backingBlack) {
            	myRobot->currState = ARobot::BLACKBACK;
            }
        } else if (c == 'd') {
            myRobot->currState = ARobot::IDLE; //IDLE
       	} else {}
		mCnt++;
    }
	return;
}

void UartRx::storeIMU(char* buf) {
    IMUData curr_imu;
    curr_imu.storeCommand(buf);
    myRobot->imuParseList.push(curr_imu); //push imu data
}

void UartRx::storeRange(char* buf) {
    RangeData curr_range(myRobot);
    curr_range.storeCommand(buf);
    myRobot->rangeParseList.push(curr_range); //push range data
}

void UartRx::storeTemp(char* buf) {
    TempData curr_temp;
    curr_temp.storeCommand(buf);
    myRobot->tempParseList.push(curr_temp); //push temp data
}

void UartRx::storeLight(char* buf) {
    LightData curr_light;
    curr_light.storeCommand(buf, myRobot->getBlackThresh(), myRobot->getSilverThresh());
    myRobot->lightParseList.push(curr_light); //push light data
}

void UartRx::storeScan(char* buf) {
	RangeData curr_range(myRobot);
	curr_range.storeCommand(buf);
	myRobot->scanParseList.push(curr_range);
}

bool UartRx::setLogFile(const char *filedir, const char* logname)
{
	if(filedir == NULL || logname == NULL)
		return false;
	
	std::string cur_date = getCurDate();
	mLogName = std::string(filedir) + "/" + std::string(logname) + "_" + cur_date;
printf("log file name = %s\n", mLogName.c_str());	
	return true;
}

bool UartRx::runLogFile()
{
	int32_t b = 100000;
	if(mLogName.empty())
		return false;

	mNum = mCnt/b;
	int32_t res = mCnt%b;
	
	if(!mpFile) {
		std::string filename = mLogName + "_" + std::to_string(mNum) + ".txt";
		mpFile = fopen(filename.c_str(), "a+");
		if(mpFile == NULL)
			return false;
		fprintf(mpFile, "%s\n\n", getCurDate().c_str());	
	}
	else if(mCnt > 0 && res == 0) {
		fclose(mpFile);
        mpFile = NULL;
		std::string filename = mLogName + "_" + std::to_string(mNum) + ".txt";
		mpFile = fopen(filename.c_str(), "a+");
		if(mpFile == NULL)
			return false;		
		fprintf(mpFile, "\n%s\n\n", getCurDate().c_str());	
	}
	
	fprintf(mpFile, "%s\n", mBuf);	
	
	return true;
}

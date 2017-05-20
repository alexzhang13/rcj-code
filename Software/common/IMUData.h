#ifndef _IMU_DATA_H_
#define _IMU_DATA_H_

#include <stdint.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include "MadgwickAHRS.h"


class IMUData {
public:

	typedef struct {
		uint32_t tstamp; //timestamp
		char id; //always comes out as i for imu
		float ax; //2g: scale factor: 16384
		float ay; //2g: scale factor: 16384
		float az; //2g: scale factor: 16384
		float gx; //250dps: scale factor: 131 
		float gy; //250dps: scale factor: 131 
		float gz; //250dps: scale factor: 131 
	}IMU_DataType;
	float m_roll; // 
	float m_pitch; //
	float m_yaw;  //

	IMUData();
	~IMUData();

	int storeCommand(char* buf);
	int getTStamp();
	int parseData();
	int runFilter();

private:
	int prev_tstamp;
	IMU_DataType data;
	char m_command[128]; //stored command
};

#endif // !_IMU_DATA_H_

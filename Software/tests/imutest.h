#ifndef _IMU_TEST_H_
#define _IMU_TEST_H_

#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include "MadgwickAHRS.h"


class ImuTest {
public:

	typedef struct {
		uint32_t tstamp;
		float ax; //2g: scale factor: 16384
		float ay; //2g: scale factor: 16384
		float az; //2g: scale factor: 16384
		float gx; //250dps: scale factor: 131 
		float gy; //250dps: scale factor: 131 
		float gz; //250dps: scale factor: 131 
		float mx;
		float my;
		float mz;
	}IMU_DataType;

	ImuTest();
	~ImuTest();

	int readData(const char *fname);
	int runFilter();

private:
	std::vector<IMU_DataType> m_data;
	Madgwick madgwick;
	float m_roll; // 
	float m_pitch; //
	float m_yaw;  //

};

#endif // !_IMU_TEST_H_
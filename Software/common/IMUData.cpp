#include "IMUData.h"
#include "MadgwickAHRS.h"
#include <stdio.h>
#include <string.h>

static Madgwick madgwick;

IMUData::IMUData()
{
	memset(m_command,'\0', 128);
	//madgwick.begin(float sampleFrequency);
	//m_data.clear();
}

IMUData::~IMUData()
{
	//m_data.clear();
}

int IMUData::storeCommand(char* buf) {
	memcpy(m_command, buf, 64);
}

int IMUData::parseData(int prev_TStamp)
{
	sscanf(m_command, "%d %c %f %f %f %f %f %f", &data.tstamp, &data.id, &data.ax, &data.ay, &data.az, &data.gx, &data.gy, &data.gz);
	prev_tstamp = prev_TStamp; //set previous
	//data.az *= -1; //inverted
	return 0;
}

int IMUData::getTStamp() {
	return data.tstamp;
}

int IMUData::runFilter()
{
	/*if(prev_tstamp != -1) {
        madgwick.set((float)data.tstamp-(float)prev_tstamp/1000); //take current in parsing timestamp minus data list most updated (previous) and get difference
    } else {
    	madgwick.set(0.026);
    }*/
	madgwick.updateIMU(data.gx/131, data.gy/131, data.gz/131, data.ax/16384, data.ay/16384, data.az/16384);
	m_pitch = madgwick.getRoll(); //inverted
	m_roll = madgwick.getPitch(); //inverted
	m_yaw = madgwick.getYaw();

	//printf("%d %d", data.tstamp, prev_tstamp);
	printf("Timestamp: %i Roll: %f Pitch: %f Yaw: %f\n", data.tstamp,  m_roll, m_pitch, m_yaw);
	//printf("Timestamp: %i Ax: %f Ay: %f Az: %f\n", data.tstamp, data.ax, data.ay, data.az);
	return 0;
}

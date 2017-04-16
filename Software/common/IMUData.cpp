#include "IMUData.h"
#include "MadgwickAHRS.h"


IMUData::IMUData()
{
	//m_data.clear();
}

IMUData::~IMUData()
{
	//m_data.clear();
}

int IMUData::parseData(char* buf)
{
	sscanf(buf, "%d %c %f %f %f %f %f %f", &data.tstamp, &data.id, &data.ax, &data.ay, &data.az, &data.gx, &data.gy, &data.gz);
	return 0;
}

int IMUData::runFilter()
{
	madgwick.updateIMU(data.gx/131, data.gy/131, data.gz/131, data.ax/16384, data.ay/16384, data.az/16384);
	m_pitch = madgwick.getRoll(); //inverted
	m_roll = madgwick.getPitch(); //inverted
	m_yaw = madgwick.getYaw();

	//printf("Timestamp: %i Roll: %f Pitch: %f Yaw: %f\n", data.tstamp,  m_roll, m_pitch, m_yaw);
	
	return 0;
}
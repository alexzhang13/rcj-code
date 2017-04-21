#include "imutest.h"
#include "MadgwickAHRS.h"


ImuTest::ImuTest()
{
	m_data.clear();
}

ImuTest::~ImuTest()
{
	m_data.clear();
}

int ImuTest::readData(const char *fname)
{
	IMU_DataType data;
	int32_t line_nums = 0;
	int ch;
	FILE *data_file;


	if (fname == NULL)
		return -1;

	data_file = fopen(fname, "r");

	while (EOF != (ch = getc(data_file)))
		if ('\n' == ch)
			++line_nums;
	rewind(data_file);
	for (int i = 0; i < line_nums; i++) {
		// read per line
		fscanf(data_file, "%i %f %f %f %f %f %f", &data.tstamp, &data.ax, &data.ay, &data.az, &data.gx, &data.gy, &data.gz/*, &data.mx, &data.my, &data.mz*/);
		m_data.push_back(data);
	}
	fclose(data_file);
	return 0;
}
int ImuTest::runFilter()
{
	if (m_data.size() == 0)
		return -1;

	for (int32_t i = 0; i < m_data.size(); i++) {
		//gx and gy are inverted
		madgwick.updateIMU(m_data[i].gx/131, m_data[i].gy/131, m_data[i].gz/131, m_data[i].ax/16384, m_data[i].ay/16384, m_data[i].az/16384);
		m_roll = madgwick.getRoll();
		m_pitch = madgwick.getPitch();
		m_yaw = madgwick.getYaw();

		printf("Timestamp: %i Roll: %f Pitch: %f Yaw: %f\n", m_data[i].tstamp,  m_roll, m_pitch, m_yaw);

	}
	return 0;
}

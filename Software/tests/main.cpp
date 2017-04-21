#include "imutest.h"


int main(int argc, char *argv[])
{
	int ret = 0;
	ImuTest it;
	const char* datafile = "C:/projects/StormingRobots2017/Data/imu_data/not_moving2.txt";

	it.readData(datafile);
	it.runFilter();

	return ret;
}
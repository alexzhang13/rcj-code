#include "TempData.h"


TempData::TempData()
{
	
}

TempData::~TempData()
{
	
}

int IMUData::storeCommand(char* buf) {
	command = buf;
}

int TempData::parseData()
{
	sscanf(command, "%f %c %f %f", &data.tstamp, &data.id, &data.tmpR, &data.tmpL);
	return 0;
}

int TempData::checkTemp()
{
	if(data.tmpL > threshold) {

	}
	if (data.tmpR > threshold) {
		
	}

	return 0;
}
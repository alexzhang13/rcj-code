#include "TempData.h"


TempData::TempData()
{
	
}

TempData::~TempData()
{
	
}

int TempData::storeCommand(char* buf, float threshLeft, float threshRight) {
	command = buf;
	thresholdL = threshLeft;
	thresholdR = threshRight;
}

int TempData::parseData()
{
	sscanf(command, "%d %c %f %f", &data.tstamp, &data.id, &data.tmpR, &data.tmpL);
	return 0;
}

int TempData::checkTemp()
{
	if(data.tmpL > thresholdL) {
		return 1;
	}
	if (data.tmpR > thresholdR) {
		return 2;
	}
	return 0;
}
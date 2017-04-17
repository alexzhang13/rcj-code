#include "TempData.h"


TempData::TempData()
{
	
}

TempData::~TempData()
{
	
}

int TempData::parseData(char* buf)
{
	sscanf(buf, "%f %c %f %f", &data.tstamp, &data.id, &data.tmpR, &data.tmpL);
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
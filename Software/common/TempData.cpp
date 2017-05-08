#include "TempData.h"
#include <stdio.h>
#include <string.h>

TempData::TempData()
{
	memset(m_command,'\0', 128);
}

TempData::~TempData()
{
	
}

int TempData::storeCommand(char* buf, float threshLeft, float threshRight) {
	memcpy(m_command, buf, 64);
	thresholdL = threshLeft;
	thresholdR = threshRight;
}

int TempData::parseData()
{
	sscanf(m_command, "%d %c %f %f", &data.tstamp, &data.id, &data.tmpR, &data.tmpL);
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

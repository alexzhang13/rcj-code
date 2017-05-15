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

int TempData::storeCommand(char* buf) {
	memcpy(m_command, buf, 64);
}

int TempData::parseData()
{
	sscanf(m_command, "%d %c %f %f", &data.tstamp, &data.id, &data.tmpR, &data.tmpL);
	return 0;
}

int TempData::getRightTemp()
{
	return data.tmpR;
}

int TempData::getLeftTemp()
{
	return data.tmpL;
}

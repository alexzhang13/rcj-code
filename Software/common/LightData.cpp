#include "LightData.h"
#include <stdio.h>
#include <string.h>

LightData::LightData()
{
	memset(m_command,'\0', 128);
}

LightData::~LightData()
{
	
}

int LightData::storeCommand(char* buf, int thresh_black, int thresh_silver, int thresh_white) {
	memcpy(m_command, buf, 64);
	threshold_black = thresh_black;
	threshold_silver = thresh_silver;
	threshold_white = thresh_white;
}

int LightData::parseData()
{
	sscanf(m_command, "%d %c %f", &data.tstamp, &data.id, &data.l_reading);
	return 0;
}

int LightData::checkLight()
{
	if(data.l_reading < threshold_black) {
		//printf("black");
		return 1;
	} else if (data.l_reading > threshold_silver) {
		//printf("silver");
		return 2;
	}
	return 0;
}

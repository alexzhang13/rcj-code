#include "../_headers/LightData.h"

#include <stdio.h>
#include <string.h>

LightData::LightData()
{
	memset(m_command,'\0', 128);
	threshold_silver=0;
	threshold_black=0;
}

LightData::~LightData()
{
	
}

void LightData::storeCommand(char* buf, int thresh_black, int thresh_silver) {
	memcpy(m_command, buf, 64);
	threshold_black = thresh_black;
	threshold_silver = thresh_silver;
}

int LightData::parseData()
{
	sscanf(m_command, "%d %c %f", &data.tstamp, &data.id, &data.l_reading);
	return 0;
}

int LightData::checkLight()
{
	printf("%f", data.l_reading);
	if(data.l_reading < threshold_black) {
		printf(": Black\n");
		return 1;
	} else if (data.l_reading < threshold_silver) {
		printf(": Silver\n");
		return 2;
	} else {
		printf(": White\n");
		return 0;
	}
	return 0;
}

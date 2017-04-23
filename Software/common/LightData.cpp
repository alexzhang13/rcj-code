#include "LightData.h"


LightData::LightData()
{
	
}

LightData::~LightData()
{
	
}

int IMUData::storeCommand(char* buf) {
	command = buf;
	//threshold_black = thresh_black;
	//threshold_silver = thresh_silver;
}

int LightData::parseData()
{
	sscanf(command, "%f %c %f", &data.tstamp, &data.id, &data.l_reading);
	return 0;
}

int LightData::checkLight()
{
	if(data.l_reading > threshold_black) {
		printf("black");
		return 1;
	} else if (data.l_reading > threshold_silver) {
		printf("silver");
		return 2;
	}
	return 0;
}
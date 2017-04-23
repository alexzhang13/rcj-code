#ifndef _LIGHT_DATA_H_
#define _LIGHT_DATA_H_

#include <stdio.h>
#include <stdint.h>
#include <string>
#include <iostream>
#include <vector>

class IMUData;
class LightData {
public:

	typedef struct {
		uint16_t tstamp; //timestamp
		char id; //always comes out as t for temp
		float l_reading;
	}Light_DataType;

	LightData();
	~LightData();

	int storeCommand(char* buf);
	int parseData();
	int checkLight();

private:
	char* command;
	Light_DataType data;
	float threshold_black;
	float threshold_silver;
};

#endif // !_TEMP_DATA_H_
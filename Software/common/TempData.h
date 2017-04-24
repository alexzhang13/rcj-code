#ifndef _TEMP_DATA_H_
#define _TEMP_DATA_H_

#include <stdio.h>
#include <stdint.h>
#include <string>
#include <iostream>
#include <vector>


class TempData {
public:

	typedef struct {
		uint32_t tstamp; //timestamp
		char id; //always comes out as t for temp
		float tmpR; //0x40
		float tmpL; //0x41 (AD0)
	}TMP_DataType;

	TempData();
	~TempData();

	int storeCommand(char* buf);
	int parseData();
	int checkTemp();

private:
	TMP_DataType data;
	float threshold;
	char* command;
};

#endif // !_TEMP_DATA_H_
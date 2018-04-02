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

	void storeCommand(char* buf);
	int parseData();
	int getRightTemp();
	int getLeftTemp();

private:
	TMP_DataType data;
	char m_command[128];
};

#endif // !_TEMP_DATA_H_

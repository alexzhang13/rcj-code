#ifndef _RANGE_DATA_H_
#define _RANGE_DATA_H_

#include <stdio.h>
#include <math.h>
#include <string>
#include <iostream>
#include <vector>


class RangeData {
public:

	typedef struct {
		uint32_t tstamp; //timestamp
		char id; //always comes out as r for range
		uint16_t angle; //0 unless robot is turning
		float laserL_a; //reading from the long laser facing forward
		float laserL_b; //reading from the long laser facing backwards
		float laserS_a; //reading from the short laser facing right
		float laserS_b; //reading from the short laser facing left
		int dir = 0; //n = 0, e = 1, s = 2, w = 3
	}Range_DataType;
	typedef struct {
		float x;
		float y;
		bool x_flag; //if valid
		bool y_flag; //if valid point
	}Range_Coord;

	RangeData();
	~RangeData();

	parseData(char* buf);
	int getPosition();

private:
	Range_DataType data;
	Range_Coord coord;
	uint8_t x_count = 0; //count if the lasers don't see anything
	uint8_t y_count = 0;
	float distance[4];
};

#endif // !_TEMP_DATA_H_
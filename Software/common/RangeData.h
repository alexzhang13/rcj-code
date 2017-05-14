#ifndef _RANGE_DATA_H_
#define _RANGE_DATA_H_

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string>
#include <iostream>
#include <vector>
#include "ARobot.h"

class ARobot;
class RangeData {
public:

	typedef struct {
		uint32_t tstamp; //timestamp
		char id; //always comes out as r for range
		int angle; //0 unless robot is turning
		float laserL_a; //reading from the long laser facing forward
		float laserL_b; //reading from the long laser facing backwards
		float laserS_a; //reading from the short laser facing right
		float laserS_b; //reading from the short laser facing left
		int dir; //n = 0, e = 1, s = 2, w = 3
	}Range_DataType;

	typedef struct {
		float x;
		float y;
		float x_glob; //from (0, 0) -> (300, 300)
		float y_glob; //from (0, 0) -> (300, 300)
		bool x_flag; //if valid
		bool y_flag; //if valid point
	}Range_Coord;

	typedef struct {
		int16_t wallN; //how many cells away is a wall
		int16_t wallE;
		int16_t wallS;
		int16_t wallW;
	}Wall_Dist;

	typedef struct {
		uint32_t tstamp; //timestamp
		char id;
		int16_t angle;
		float readingN;
		float readingE;
		float readingS;
		float readingW;
	}Scan_DataType;

	RangeData(ARobot *robot);
	~RangeData();

	int storeCommand(char* buf);
	int parseData();
	int getPosition();
	int getScan();

	Scan_DataType scan;
	Range_DataType data;
	Range_Coord coord;
	Wall_Dist walls;

protected:
	ARobot *myRobot;
private:
	uint8_t x_count; //count if the lasers don't see anything
	uint8_t y_count;
	float distance[4];
	char m_command[128]; //stored command
	float temp_range[4]; //temporary number
	int temp_dist; //temporary number
};

#endif // !_TEMP_DATA_H_

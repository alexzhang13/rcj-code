#ifndef _CELLMAP_GEN_H_
#define _CELLMAP_GEN_H_

#include "navigate_defs.h"
#include <string>
#include <vector>
#include <map>
#include <stdint.h>
#include "cell.h"
#include "floormap.h"
#include "mazemap.h"
#include <opencv2/opencv.hpp>
#include <xml/tinyxml.h>


class NAVIGATE_EXPORT MazeMapGen : public MazeMaps {
public:

	MazeMapGen();
	~MazeMapGen();

	int32_t initMap_floor0(int32_t width, int32_t height, int32_t checkpt, int32_t victims, int32_t nmovables, int32_t obstables, float walls);
	int32_t initMap_floor1(int32_t width, int32_t height, int32_t checkpt, int32_t victims, int32_t nmovables, int32_t obstables, float walls);

	int32_t generateSimulateMap(int32_t floor_num);

protected:
	void addStairCell(int32_t floor_num);
	static bool sortbyCellIndex(MazeCell &node1, MazeCell &node2);
private:

};

#endif
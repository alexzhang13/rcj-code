#ifndef _MAZEMAP_H_
#define _MAZEMAP_H_

#include "navigate_defs.h"
#include <string>
#include <vector>
#include <map>
#include <stdint.h>
#include "cell.h"
#include "floormap.h"
#include <opencv2/opencv.hpp>
#include <tinyxml.h>
#include "greedy_dijkstra.h"

class NAVIGATE_EXPORT MazeMaps {
public:

	typedef struct {
		int32_t grid_w;
		int32_t grid_h;
		int32_t checkpt;
		int32_t victims;
		int32_t nmovables;
		int32_t obstables;
		bool stair;
		float wall_percent;
	} FloorMapStatus;

	// constructor
	MazeMaps();
	// destructor
	~MazeMaps();

	void updateParams(int32_t home_floor=0, int32_t numofFloors=2);

	int32_t displayPhysicalMap(int32_t floor_num, std::vector<GreedyDijkstra::DistInfo>* trace = NULL);
	int32_t savePhysicalMap(const char* out_dir, const char* name, int32_t floor_num);

	int32_t writeXmlMap(const char* out_dir, const char* name);
	int32_t readXmlMap(const char* out_dir, const char* name);


	inline MazeFloorMap *getFloorMap(int32_t floor_num) { return &m_floormap[floor_num]; }
	inline int32_t getNumOfFloors() { return m_floors;}
	inline int32_t getHomeFloorNum() { return m_home_floor;}
	inline int32_t getHomeCellIndex() { return m_home_cell_index;}
	inline int32_t getFloorNums() { return m_floors;}
	inline int32_t getCurFloorNum() { return m_cur_floor;}
	inline int32_t getCurCellIndex() { return m_cur_cell_index;}

	inline std::vector<GreedyDijkstra::DistInfo> *getTracedRoute(int32_t floor_num) { return &m_route_trace[floor_num];}

protected:

	void drawRectangle(cv::Mat &img, int32_t floor_num, MazeCell *cell=NULL);
	void fillRectangle(cv::Mat &img, int32_t floor_num, MazeCell *cell=NULL);
	void drawLines(cv::Mat &img, int32_t floor_num, std::vector<int32_t> *trace_pts=NULL);

	int32_t getWallColor(MazeCell::WallProp ct, cv::Scalar &color);
	int32_t getCellColor(MazeCell::CellType ct, cv::Scalar &color);

	int32_t m_cur_floor;
	int32_t m_cur_cell_index;
	int32_t m_home_cell_index;
	int32_t m_home_floor;
	int32_t m_floors;

	MazeFloorMap m_floormap[2];
	FloorMapStatus m_floor[2];
	cv::Mat m_disp_img[2];
	std::vector<GreedyDijkstra::DistInfo> m_route_trace[2];
	int32_t m_disp_cell_size;
};

#endif

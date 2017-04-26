#ifndef _NAVIGATE_H_
#define _NAVIGATE_H_


#include "navigate_defs.h"
#include "mazemap.h"
#include "mazemap_gen.h"
#include "greedy_dijkstra.h"


class NAVIGATE_EXPORT Navigate2D {

public:
	//! constructor
	Navigate2D();
	//! destructor
	~Navigate2D();

	// read in ground truth maps
	int32_t readChkPtMaps(const char* out_dir, const char* filename);

	//! set home cell
	int32_t setHomeCell(int floor_num, MazeCell::NavDir heading);

	//! set home cell
	int32_t setStairCell(int floor_num, MazeCell *staircell);

	//! configure current cell based on sensor info
	int32_t configureCurCell();

	//! update local map based on parsed sensor data
	int32_t updateLocalMap(); 

	//! navigation planning
	int32_t navigatePlanning();

	//! navigation
	int32_t navigation2D();

	//! display ground truth map 
	bool displayGtMap(int32_t floor_num);

	//! display current local map
	bool displayLocalMap();

	//! displat current local map with way points
	bool displayRouteMap();

	//! detect local cells in simulation
	int32_t detectLocalCells();


protected:
	void resetGraphMatrix();
	void allocateGraphMatrix(int32_t cellsize);
	int32_t generateGraphMatrix();
private:

	std::string m_map_file;
	int32_t m_floors;
	int32_t m_home_floor_num;
	MazeCell::NavDir m_chkpt_heading;
	std::vector<int32_t> m_newcell_list;
	int32_t **m_graph_matrix;
	int32_t m_graph_size;
	MazeMaps m_navigateMaps;
	int32_t m_home_cell_index; 
	int32_t m_cur_floor_index;
	int32_t m_cur_cell_index;
	GreedyDijkstra m_dijkstra;
	GreedyDijkstra::DistInfo m_next_cell;
};





#endif
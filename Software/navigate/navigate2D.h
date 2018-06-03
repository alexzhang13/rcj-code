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
	virtual int32_t readChkPtMaps(const char* out_dir, const char* filename);

	//! set home cell
	virtual int32_t setHomeCell(int floor_num, MazeCell::NavDir heading);

	//! set home cell
	virtual int32_t setStairCell(int floor_num, MazeCell *staircell);

	//! configure current cell based on sensor info
	virtual int32_t configureCurCell(MazeCell *sensor_info);

	//! detect local cells from sensor measures
	virtual int32_t detectLocalCells(std::vector<MazeCell> &next_cell_list);

	//! update local map based on parsed sensor data
	virtual int32_t updateLocalMap(); 

	//! navigation planning
        virtual int32_t navigatePlanning(bool returnStart);

	//! navigation
	virtual int32_t navigation2D();

	//! display current local map
	virtual bool displayLocalMap();

	//! displat current local map with way points
	virtual bool displayRouteMap();

	//! write the current map config file
	virtual bool writeMapFile(const char* out_dir, const char* filename);

	//! get current time as a string
	virtual std::string getCurTime();

	inline MazeCell *getCurrentCell() {return m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(m_cur_cell_index);}
	inline MazeCell *getCellbyIndex(int32_t indx) {return m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(indx);}
	inline MazeMaps *getNavigateMaps() { return &m_navigateMaps;}
	inline int32_t getCurrentFloorIndex() { return m_cur_floor_index; }
	inline int32_t getCurrentCellIndex() { return m_cur_cell_index; }
	inline void setCurrentFloorIndex(int32_t indx) { m_cur_floor_index = indx; }
	inline void setCurrentCellIndex(int32_t indx) { m_cur_cell_index = indx; }
	inline GreedyDijkstra::DistInfo *getNextCell() { return &m_next_cell;}
protected:
	void resetGraphMatrix();
	void allocateGraphMatrix(int32_t cellsize);
	int32_t generateGraphMatrix();

protected:
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

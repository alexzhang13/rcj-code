#ifndef _CELLMAP_H_
#define _CELLMAP_H_

#include "navigate_defs.h"
#include <string>
#include <vector>
#include <map>
#include <stdint.h>
#include "cell.h"

class NAVIGATE_EXPORT MazeFloorMap {
public:
	// constructor
	MazeFloorMap();
	// destructor
	~MazeFloorMap();

	// this is the first cell
	int32_t allocateHomeCell(std::string basecell);

	int32_t findNewCell(int idx_x, int idx_y); // return cell index if new cell is found and added

	void addNewCell(int idx_x, int idx_y, float cell_width); // add a cell 

	void addNewCell(MazeCell &cell);

	int32_t updateCell(int idx_x, int idx_y);

	int32_t updateCellArray();

	int32_t connectCells();

	// pre-allocate cells - only for map simulation
	int32_t preallocCells(int32_t grid_width, int32_t grid_height);

	// reset cell map
	void resetMap();

	inline bool isHomeCell() { return m_is_home;}
	inline void setHomeCellFlag(bool flag) {m_is_home = flag;}
	inline int32_t getHomeCellNum() { return m_home_cell_index;}
	inline void setHomeCellNum(int32_t num) { m_home_cell_index = num; if(m_cell_list.size() > 0)  m_home_cell = &m_cell_list[num];}
	inline void setHomeCell(MazeCell *cell) { m_home_cell = cell;} 
	inline MazeCell *getHomeCell() { return m_home_cell;}
	inline MazeCell *getCurrentCell() { return m_cur_cell;}
	inline int32_t getCurCellIndex() { return m_cur_cell_index;}
	inline void setCurCellIndex(int32_t indx) { m_cur_cell_index = indx; if(m_cell_list.size() > 0)  m_cur_cell = &m_cell_list[indx];}
	inline MazeCell* getStairCell() { return m_stair_cell;}
	inline void setStairCell(MazeCell *cell) { m_stair_cell = cell;} 

	inline int32_t getCellSize() { return (int32_t)m_cell_list.size();}
	inline MazeCell *getCell(int32_t i) { if(i < 0) return NULL; else return &m_cell_list[i];}
	inline std::vector<MazeCell> *getCellList() { return &m_cell_list; }
	inline std::vector<int32_t> *getVisitedList() { return &m_visited_list; }
	inline std::vector<int32_t> *getToBeVisitedList() { return &m_to_be_visited_list; }
	inline std::vector<int32_t> *getCheckPtList() { return &m_checkPtList;}
	inline std::vector<int32_t> *getUnknownList() { return &m_unknown_list;}

	inline void setMapHsize(float hsize) { m_map_hsize = hsize;}
	inline void setMapVsize(float vsize) { m_map_vsize = vsize;}
	inline float getMapHsize(void) { return m_map_hsize;}
	inline float getMapVsize(void) { return m_map_vsize;}

	inline void setGridHsize(int32_t hsize) { m_grid_wsize = hsize;}
	inline void setGridVsize(int32_t vsize) { m_grid_hsize = vsize;}
	inline int32_t getGridHsize(void) { return m_grid_wsize;}
	inline int32_t getGridVsize(void) { return m_grid_hsize;}

	inline void setStairCellIndex(int32_t indx) { m_stair_cell_index = indx;  if(m_cell_list.size() > 0) m_stair_cell = &m_cell_list[indx];}
	inline int32_t getStairCellIndex() { return m_stair_cell_index;}
	inline void setLatestChkPtCellIndex(int32_t indx) { m_latest_checkpt_index = indx; if(m_cell_list.size() > 0)  m_latest_checkpt_cell = &m_cell_list[indx];}
	inline int32_t getLatestChkPtCellIndex() { return m_latest_checkpt_index;}


protected:
	bool m_initialized; 
	bool m_is_home;
	int32_t m_home_cell_index;
	int32_t m_cur_cell_index;
	int32_t m_stair_cell_index;
	int32_t m_latest_checkpt_index;
	std::vector<MazeCell> m_cell_list; // the cell is listed by the order of cell number
	std::vector<int32_t> m_visited_list;
	std::vector<int32_t> m_to_be_visited_list;
	std::vector<int32_t> m_unknown_list;
	std::vector<int32_t> m_checkPtList;

	MazeCell ***m_2dcell_array;
	MazeCell *m_home_cell;
	MazeCell *m_latest_checkpt_cell;
	MazeCell *m_stair_cell;
	MazeCell *m_cur_cell;
	int32_t m_grid_wsize;
	int32_t m_grid_hsize;
	float m_map_hsize;
	float m_map_vsize;
};


/////////////////////////////////////////
#endif

#ifndef _NAVIGATION_SIMUL_H_
#define _NAVIGATION_SIMUL_H_

#include "navigate_defs.h"
#include "mazemap.h"
#include "mazemap_gen.h"
#include "greedy_dijkstra.h"
#include "navigate2D.h"

class NAVIGATE_EXPORT NavigateSimul : public Navigate2D {

public:
	//! constructor
	NavigateSimul();
	//! destructor
	~NavigateSimul();

	// read in ground truth maps
	virtual int32_t readChkPtMaps(const char* out_dir, const char* filename);

	//! set home cell
	virtual int32_t setHomeCell(int floor_num, MazeCell::NavDir heading);

	//! set home cell
	virtual int32_t setStairCell(int floor_num, MazeCell *staircell);

	//! configure current cell based on sensor info
	virtual int32_t configureCurCell();

	//! detect local cells in simulation
	virtual int32_t detectLocalCells();

	//! update local map based on parsed sensor data
	virtual int32_t updateLocalMap(); 

	//! navigation planning
	virtual int32_t navigatePlanning();

	//! navigation
	virtual int32_t navigation2D();

	//! display ground truth map 
	virtual bool displayGtMap(int32_t floor_num);

	//! display current local map
	virtual bool displayLocalMap();

	//! displat current local map with way points
	virtual bool displayRouteMap();


protected:
	void resetGraphMatrix();
	void allocateGraphMatrix(int32_t cellsize);
	int32_t generateGraphMatrix();
	MazeMapGen m_gt_maps;
};

//////////////////////////////////////////////////////////////
#endif

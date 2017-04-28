#ifndef _CELL_H_
#define _CELL_H_

#include "navigate_defs.h"
#include <string>
#include <vector>
#include <stdint.h>
#include "globals.h"


class NAVIGATE_EXPORT MazeCell {
public:

	// location in (x,y)
	typedef struct {
		float x;
		float y;
	} xyCoord;

	typedef struct {
		int32_t x;
		int32_t y;
	} Position_2D;

	typedef enum {
		MOpen = MAZE_MIN_VAL,
		MWall = MAZE_MAX_VAL,
		MBlack = MAZE_NO_ACCESS,
		MStair = MAZE_to_STAIR,
		MObstacle = MAZE_OBSTACLE,
		MFail = MAZE_FAILED_WT,
		MUnknown = 0
	} WallProp;

	// navigation action
	typedef struct {
		float angle; // angle to turn
		float distance; // distance to travel 
	} NavTransition;

	// moving direction
	typedef enum {
		NotDecided = -1,
		navNorth = 0,
		navEast = 1,
		navSouth = 2,
		navWest = 3,
		navDirections = 4
	} NavDir;

	// cell type
	typedef struct {
		bool GAccess;   // accessible
		bool GVictim;   // hot spot - victim
		bool GNMovable; // black plate
		bool GObstacle; // obstacle in the cell
		bool GStair;    // a cell with stair
		bool GCheckPt;    // silver plate
		bool GHome;     // starting cell 
		NavDir GVictimDirection; // direction of the victim to be found
	} CellType;

	// cell history
	typedef enum {
		NotFound = 0,
		TobeVisited = 1,
		Visited = 2,
		Prohibited = -1
	} CellHistory;

	// navigation state
	typedef struct {
		NavDir nav_dir;    // direction the robot is currently facing
		xyCoord localpos;  // robot position wrt center center
		xyCoord centerpos; // cell center position wrt map origin
		CellHistory history; // cell history 
	} NavState;

	// constructor
	MazeCell();
	// destructor
	~MazeCell();

	// reset all parameters
	void reset();

	// initialize parameters
	void initParams(float cellsize);
	void setNonMovable(bool flag);
	void setObstacle(bool flag);
	void setVictimDirection(NavDir vdir);
	NavDir getVictimDirection();

	inline void setCellNum(int32_t cellnum) { m_cellNum = cellnum;}
	inline int32_t getCellNum() { return m_cellNum;}
	inline void setCellWidth(float cell_size) {m_cellsize = cell_size;}
	inline float getCellWidth() {return m_cellsize;}
	inline void setCellGrid(int32_t i, int32_t j) {m_gridxy.x=i; m_gridxy.y = j;}
	inline void getCellGrid(int32_t &i, int32_t &j) {i=m_gridxy.x; j=m_gridxy.y;}
	inline void setCenterXY(float x, float y) {m_navstate.centerpos.x = x;m_navstate.centerpos.y = y;} 
	inline void getCenterXY(float &x, float &y) {x=m_navstate.centerpos.x;y=m_navstate.centerpos.y;} 
	inline void updateNorthNeighbor(int32_t cn) { m_cellN = cn;}
	inline int32_t getNeighborCellNorth() { return m_cellN;}
	inline void updateEastNeighbor(int32_t cn) { m_cellE = cn;}
	inline int32_t getNeighborCellEast() { return m_cellE;}
	inline void updateSouthNeighbor(int32_t cn) { m_cellS = cn;}
	inline int32_t getNeighborCellSouth() { return m_cellS;}
	inline void updateWestNeighbor(int32_t cn) { m_cellW = cn;}
	inline int32_t getNeighborCellWest() { return m_cellW;}

	inline void setNavDirection(NavDir nd) {m_navstate.nav_dir = nd; }
	inline NavDir getNavDirection() { return m_navstate.nav_dir; }

	inline void setVisitStatus(CellHistory chis) {m_navstate.history = chis;}
	inline CellHistory getVisitStatus() { return m_navstate.history;}

	inline void enableCellValid(bool flag) { m_valid_cell = flag;}
	inline bool getValidCellFlag() { return m_valid_cell; }

	inline void setCellType(CellType ct) { m_celltype = ct;}
	inline CellType getCellType() { return m_celltype; }
	inline void setHomeCell(bool flag) { m_celltype.GHome = flag;}
	inline bool getHomeCell() { return m_celltype.GHome;}
	inline void setStairCell(bool flag) {m_celltype.GStair = flag;}
	inline bool getStairCell() {return m_celltype.GStair;}
	inline void setCheckPt(bool flag) { m_celltype.GCheckPt = flag;}
	inline bool getCheckPt() { return m_celltype.GCheckPt;}
	inline void setVictim(bool flag) {m_celltype.GVictim = flag; }
	inline bool getVictim() { return m_celltype.GVictim; }
	inline bool getNonMovable() { return m_celltype.GNMovable; }
	inline bool getObstacle() { return m_celltype.GObstacle; }

	inline void setWallNorth(WallProp wp) { m_wallN = wp;}
	inline WallProp getWallNorth() { return m_wallN;}
	inline void setWallEast(WallProp wp) { m_wallE = wp;}
	inline WallProp getWallEast() { return m_wallE;}
	inline void setWallSouth(WallProp wp) { m_wallS = wp;}
	inline WallProp getWallSouth() { return m_wallS;}
	inline void setWallWest(WallProp wp) { m_wallW = wp;}
	inline WallProp getWallWest() { return m_wallW;}

	inline void setDebugLinkCell(MazeCell *dcell) { m_debug_link = dcell; }
	inline MazeCell *getDebugLinkCell() { return m_debug_link; }

protected:


private:
	float m_cellsize;
	CellType m_celltype; // cell types 
	Position_2D m_gridxy; // grid (x,y) in the map
	NavState m_navstate;
	NavTransition m_transit; 
	int32_t m_cellNum;
	bool m_valid_cell;

	int32_t m_cellN;
	int32_t m_cellE;
	int32_t m_cellS;
	int32_t m_cellW;
	WallProp m_wallN;
	WallProp m_wallE;
	WallProp m_wallS;
	WallProp m_wallW;

	MazeCell *m_debug_link;
};

#endif
#include "cell.h"

// constructor
MazeCell::MazeCell()
{
	reset();
}

// destructor
MazeCell::~MazeCell()
{
	reset();
}

void MazeCell::reset()
{
	m_cellsize = 300.0f; // 300mm
	m_cellNum = -1; // not assigned
	m_cellN = -1;
	m_cellE = -1;
	m_cellS = -1;
	m_cellW = -1;
	m_wallN = MUnknown;
	m_wallE = MUnknown;
	m_wallS = MUnknown;
	m_wallW = MUnknown;
	m_celltype.GCheckPt = false;
	m_celltype.GAccess = false;
	m_celltype.GHome = false;
	m_celltype.GNMovable = false;
	m_celltype.GObstacle = false;
	m_celltype.GStair = false;
	m_celltype.GVictim = false;
	m_celltype.GVictimDirection = NotDecided;
	m_gridxy.x = 0;
	m_gridxy.y = 0;
	m_navstate.localpos.x = 0.0f;
	m_navstate.localpos.y = 0.0f;
	m_navstate.history = NotFound;
	m_navstate.centerpos.x = 0.0f;
	m_navstate.centerpos.y = 0.0f;
	m_navstate.nav_dir = NotDecided;
	m_valid_cell = false;
	m_debug_link = NULL;
}


void MazeCell::initParams(float cellsize)
{
	m_cellsize = cellsize;
}

void MazeCell::setNonMovable(bool flag) 
{
	m_celltype.GNMovable = flag; 
	if(m_celltype.GNMovable) {
		if(getWallNorth() != MWall)
			setWallNorth(MBlack);
		if(getWallEast() != MWall)
			setWallEast(MBlack);
		if(getWallSouth() != MWall)
			setWallSouth(MBlack);
		if(getWallWest() != MWall)
			setWallWest(MBlack);
	}
	else {
#if 0
		if(getWallNorth() != MWall)
			setWallNorth(MOpen);
		if(getWallEast() != MWall)
			setWallEast(MOpen);
		if(getWallSouth() != MWall)
			setWallSouth(MOpen);
		if(getWallWest() != MWall)
			setWallWest(MOpen);
#endif
	}
}

void MazeCell::setObstacle(bool flag) 
{
	m_celltype.GObstacle = flag; 
	if(m_celltype.GObstacle) {
		if(getWallNorth() != MWall)
			setWallNorth(MObstacle);
		if(getWallEast() != MWall)
			setWallEast(MObstacle);
		if(getWallSouth() != MWall)
			setWallSouth(MObstacle);
		if(getWallWest() != MWall)
			setWallWest(MObstacle);
	}
	else {
#if 0
		if(getWallNorth() != MWall)
			setWallNorth(MOpen);
		if(getWallEast() != MWall)
			setWallEast(MOpen);
		if(getWallSouth() != MWall)
			setWallSouth(MOpen);
		if(getWallWest() != MWall)
			setWallWest(MOpen);
#endif
	}
}

void MazeCell::setVictimDirection(MazeCell::NavDir vdir)
{
	if(getVictimDirection() == NotDecided) {
		m_celltype.GVictimDirection = vdir;
	}
	return;
}

MazeCell::NavDir MazeCell::getVictimDirection()
{
	return m_celltype.GVictimDirection;
}
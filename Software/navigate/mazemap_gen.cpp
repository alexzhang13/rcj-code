#include "mazemap_gen.h"
#include <assert.h>


bool MazeMapGen::sortbyCellIndex(MazeCell &node1, MazeCell &node2)
{
	int32_t dist1 = node1.getCellNum();
	int32_t dist2 = node2.getCellNum();
	return (dist1 < dist2);
}

MazeMapGen::MazeMapGen()
{
}

MazeMapGen::~MazeMapGen()
{
}


int32_t MazeMapGen::initMap_floor0(int32_t width, int32_t height, int32_t checkpt, int32_t victims, int32_t nmovables, int32_t obstables, float walls)
{
	m_floor[0].grid_w = width;
	m_floor[0].grid_h = height;
	m_floor[0].checkpt = checkpt;
	m_floor[0].victims = victims;
	m_floor[0].nmovables = nmovables;
	m_floor[0].obstables = obstables;
	m_floor[0].wall_percent = walls;

	m_floormap[0].preallocCells(m_floor[0].grid_w, m_floor[0].grid_h);
	m_floormap[0].updateCellArray();

	std::sort(m_floormap[0].getCellList()->begin(), m_floormap[0].getCellList()->end(), sortbyCellIndex);
	return 0;
}

int32_t MazeMapGen::initMap_floor1(int32_t width, int32_t height, int32_t checkpt, int32_t victims, int32_t nmovables, int32_t obstables, float walls)
{
	m_floor[1].grid_w = width;
	m_floor[1].grid_h = height;
	m_floor[1].checkpt = checkpt;
	m_floor[1].victims = victims;
	m_floor[1].nmovables = nmovables;
	m_floor[1].obstables = obstables;
	m_floor[1].wall_percent = walls;

	m_floormap[1].preallocCells(m_floor[1].grid_w, m_floor[1].grid_h);
	m_floormap[1].updateCellArray();

	std::sort(m_floormap[1].getCellList()->begin(), m_floormap[1].getCellList()->end(), sortbyCellIndex);
	return 0;
}

int32_t MazeMapGen::generateSimulateMap(int32_t floor_num)
{
	int32_t cellsize = m_floormap[floor_num].getCellSize();
	int32_t i, r;
	int32_t wall_thresh =(int32_t) (m_floor[floor_num].wall_percent * 100.0f);
	int32_t floor_grid_w = m_floor[floor_num].grid_w;
	int32_t floor_grid_h = m_floor[floor_num].grid_h;

	/* Intializes random number generator */
	srand((unsigned int)12); //srand((unsigned int)NULL);

	m_floormap[floor_num].connectCells();

	// use random number generator to assign walls and openings
	for(i = 0; i < cellsize; i++) {
		MazeCell *cell = m_floormap[floor_num].getCell(i);
		if(cell->getWallNorth() == MazeCell::MUnknown) {
			r = rand() % (100);
			if(r < wall_thresh) {
				cell->setWallNorth(MazeCell::MWall);
				MazeCell * northcell = m_floormap[floor_num].getCell(cell->getNeighborCellNorth());
				if(northcell != NULL)
					northcell->setWallSouth(MazeCell::MWall);
			}
			else {
				cell->setWallNorth(MazeCell::MOpen);
				MazeCell * northcell = m_floormap[floor_num].getCell(cell->getNeighborCellNorth());
				if(northcell != NULL)
					northcell->setWallSouth(MazeCell::MOpen);
			}
		}
		if(cell->getWallEast() == MazeCell::MUnknown) {
			r = rand() % (100);
			if(r < wall_thresh) {
				cell->setWallEast(MazeCell::MWall);
				MazeCell * eastcell = m_floormap[floor_num].getCell(cell->getNeighborCellEast());
				if(eastcell != NULL)
					eastcell->setWallWest(MazeCell::MWall);
			}
			else {
				cell->setWallEast(MazeCell::MOpen);
				MazeCell * eastcell = m_floormap[floor_num].getCell(cell->getNeighborCellEast());
				if(eastcell != NULL)
					eastcell->setWallWest(MazeCell::MOpen);
			}
		}
		if(cell->getWallSouth() == MazeCell::MUnknown) {
			r = rand() % (100);
			if(r < wall_thresh) {
				cell->setWallSouth(MazeCell::MWall);
				MazeCell * southcell = m_floormap[floor_num].getCell(cell->getNeighborCellSouth());
				if(southcell != NULL)
					southcell->setWallNorth(MazeCell::MWall);
			}
			else {
				cell->setWallSouth(MazeCell::MOpen);
				MazeCell * southcell = m_floormap[floor_num].getCell(cell->getNeighborCellSouth());
				if(southcell != NULL)
					southcell->setWallNorth(MazeCell::MOpen);
			}
		}
		if(cell->getWallWest() == MazeCell::MUnknown) {
			r = rand() % (100);
			if(r < wall_thresh) {
				cell->setWallWest(MazeCell::MWall);
				MazeCell * westcell = m_floormap[floor_num].getCell(cell->getNeighborCellWest());
				if(westcell != NULL)
					westcell->setWallEast(MazeCell::MWall);
			}
			else {
				cell->setWallWest(MazeCell::MOpen);
				MazeCell * westcell = m_floormap[floor_num].getCell(cell->getNeighborCellWest());
				if(westcell != NULL)
					westcell->setWallEast(MazeCell::MOpen);
			}
		}
	}

	std::vector<int32_t> rand_list;
	for(i = 0; i < 20; i++) {
		r = rand() % cellsize;
		rand_list.push_back(r);
	}

	// check points
	int32_t cnt = 0;
	for(i = 0; i < m_floor[floor_num].checkpt; i++) {
		MazeCell *cell = m_floormap[floor_num].getCell(rand_list[cnt]);
		cell->setCheckPt(true);
		cnt++;
	}

	// victims
	for(i = 0; i < m_floor[floor_num].victims; i++) {
		MazeCell *cell = m_floormap[floor_num].getCell(rand_list[cnt]);
		cell->setVictim(true);
		cell->setVictimDirection(MazeCell::NavDir(rand_list[cnt]%MazeCell::navDirections));
		cnt++;
	}

	// non-movables - black plate
	for(i = 0; i < m_floor[floor_num].nmovables; i++) {
		MazeCell *cell = m_floormap[floor_num].getCell(rand_list[cnt]);
		cell->setNonMovable(true);
		cnt++;
	}

	// obstacles
	for(i = 0; i < m_floor[floor_num].obstables; i++) {
		MazeCell *cell = m_floormap[floor_num].getCell(rand_list[cnt]);
		cell->setObstacle(true);
		cnt++;
	}

	// stair cell is a special case, which is outside of the mxn grid
	addStairCell(floor_num);

	// merge cell walls
	MazeCell::WallProp wp;
	bool flag;
	for(i = 0; i < cellsize; i++) {
		MazeCell *cell = m_floormap[floor_num].getCell(i);
		printf("cell number = %d\n", cell->getCellNum());
		wp = cell->getWallNorth();
		flag = (wp == MazeCell::MBlack || wp == MazeCell::MObstacle || wp == MazeCell::MFail);
		if(cell->getNeighborCellNorth()>=0) {
			if(wp == MazeCell::MWall)
				(m_floormap[floor_num].getCell(cell->getNeighborCellNorth()))->setWallSouth(wp);
			else if(m_floormap[floor_num].getCell(cell->getNeighborCellNorth())->getWallSouth() != MazeCell::MWall) {
				if(flag)
					m_floormap[floor_num].getCell(cell->getNeighborCellNorth())->setWallSouth(wp);
				else
					cell->setWallNorth(m_floormap[floor_num].getCell(cell->getNeighborCellNorth())->getWallSouth());
			}
		}
		wp = cell->getWallEast();
		flag = (wp == MazeCell::MBlack || wp == MazeCell::MObstacle || wp == MazeCell::MFail);
		if(cell->getNeighborCellEast()>=0) {
			if(wp == MazeCell::MWall) 
				m_floormap[floor_num].getCell(cell->getNeighborCellEast())->setWallWest(wp);
			else if(m_floormap[floor_num].getCell(cell->getNeighborCellEast())->getWallWest() != MazeCell::MWall) {
				if(flag)
					m_floormap[floor_num].getCell(cell->getNeighborCellEast())->setWallWest(wp);
				else
					cell->setWallEast(m_floormap[floor_num].getCell(cell->getNeighborCellEast())->getWallWest());
			}
		}
		wp = cell->getWallSouth();
		flag = (wp == MazeCell::MBlack || wp == MazeCell::MObstacle || wp == MazeCell::MFail);
		if(cell->getNeighborCellSouth()>=0) {
			if(wp == MazeCell::MWall ) 
				m_floormap[floor_num].getCell(cell->getNeighborCellSouth())->setWallNorth(wp);
			else if(m_floormap[floor_num].getCell(cell->getNeighborCellSouth())->getWallNorth() != MazeCell::MWall) {
				if(flag)
					m_floormap[floor_num].getCell(cell->getNeighborCellSouth())->setWallNorth(wp);
				else
					cell->setWallSouth(m_floormap[floor_num].getCell(cell->getNeighborCellSouth())->getWallNorth());
			}
		}
		wp = cell->getWallWest();
		flag = (wp == MazeCell::MBlack || wp == MazeCell::MObstacle || wp == MazeCell::MFail);
		if(cell->getNeighborCellWest()>=0) {
			if(wp == MazeCell::MWall) {
				m_floormap[floor_num].getCell(cell->getNeighborCellWest())->setWallEast(wp);
			}
			else if(m_floormap[floor_num].getCell(cell->getNeighborCellWest())->getWallEast() != MazeCell::MWall) {
				if(flag)
					m_floormap[floor_num].getCell(cell->getNeighborCellWest())->setWallEast(wp);
				else
					cell->setWallWest(m_floormap[floor_num].getCell(cell->getNeighborCellWest())->getWallEast());
			}
		}
	}

	m_floormap[floor_num].updateCellArray();

	return 0;
}

void MazeMapGen::addStairCell(int32_t floor_num)
{
	int32_t i,x,y;
	int32_t cell_nums = m_floormap[floor_num].getCellSize();
	int32_t grid_x = m_floor[floor_num].grid_w;
	int32_t grid_y = m_floor[floor_num].grid_h;
	int32_t ry = rand() % (grid_y);
	int32_t rx = -1; //grid_x; // or -1
	MazeCell normal_cell, staircell;

	normal_cell.setCellNum(cell_nums);
	normal_cell.setCellGrid(rx, ry);
	normal_cell.enableCellValid(true);

	if(rx == -1) {
		normal_cell.setWallNorth(MazeCell::MStair);
		normal_cell.setWallEast(MazeCell::MOpen);
		normal_cell.setWallSouth(MazeCell::MWall);
		normal_cell.setWallWest(MazeCell::MWall);
		for(i = 0; i < cell_nums; i++) {
			MazeCell *cell = m_floormap[floor_num].getCell(i);
			cell->getCellGrid(x,y);
			if(rx + 1 == x && ry == y) {
				cell->setWallWest(MazeCell::MOpen);
				normal_cell.updateEastNeighbor(cell->getCellNum());
				cell->updateWestNeighbor(normal_cell.getCellNum());
			}
		}
	}
	else if(rx == grid_x) {
		normal_cell.setWallNorth(MazeCell::MStair);
		normal_cell.setWallEast(MazeCell::MWall);
		normal_cell.setWallSouth(MazeCell::MWall);
		normal_cell.setWallWest(MazeCell::MOpen);
		for(i = 0; i < cell_nums; i++) {
			MazeCell *cell = m_floormap[floor_num].getCell(i);
			cell->getCellGrid(x,y);
			if(rx - 1 == x && ry == y) {
				cell->setWallEast(MazeCell::MOpen);
				normal_cell.updateWestNeighbor(cell->getCellNum());
				cell->updateEastNeighbor(normal_cell.getCellNum());
			}
		}
	}
	m_floormap[floor_num].getCellList()->push_back(normal_cell);

	// add stair as virtual cell
	ry++;
	cell_nums++;
	staircell.setCellNum(cell_nums);
	staircell.setCellGrid(rx, ry);
	staircell.setStairCell(true);
	staircell.enableCellValid(true);

	staircell.setWallNorth(MazeCell::MOpen);
	staircell.setWallEast(MazeCell::MWall);
	staircell.setWallSouth(MazeCell::MStair);
	staircell.setWallWest(MazeCell::MWall);

	staircell.updateSouthNeighbor(normal_cell.getCellNum());
	normal_cell.updateNorthNeighbor(staircell.getCellNum());
	m_floormap[floor_num].getCellList()->push_back(staircell);

	return;
}
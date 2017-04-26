#include "navigate2D.h"
#include <iostream>
#ifdef WIN32
#include <windows.h>
#include <sys/utime.h>
#include <sstream>
#include <ctime>
#else
#include <ctime>
#endif

Navigate2D::Navigate2D()
{
	/* Intializes random number generator */
	srand((unsigned int)NULL);
	m_cur_floor_index = 0;
	m_floors = 2;
	m_home_floor_num = 0;
	m_home_cell_index = 0;
	m_newcell_list.clear();
	m_graph_matrix = NULL;
	m_graph_size = 0;
	m_map_file.clear();
}

Navigate2D::~Navigate2D()
{
	m_newcell_list.clear();
	resetGraphMatrix();
}

// read in maps at the latest check point
int32_t Navigate2D::readChkPtMaps(const char* out_dir, const char* filename)
{
	m_map_file = std::string(out_dir) + std::string(filename) + ".xml";
	if(m_navigateMaps.readXmlMap(out_dir, filename) != 0) {
		printf("Not able to load any map from %s/%s\n", out_dir, filename);
		// nothing found, need to build up the map from scratch
	}
	else { // successfully reading

		m_floors = m_navigateMaps.getNumOfFloors();
		m_home_floor_num = m_navigateMaps.getHomeFloorNum();
		m_home_cell_index =m_navigateMaps.getHomeCellIndex();
		m_cur_floor_index = m_navigateMaps.getCurFloorNum();
		m_cur_cell_index = m_navigateMaps.getCurCellIndex();

		for(int32_t i = 0; i < m_floors; i++)
			m_navigateMaps.getFloorMap(i)->connectCells();
	}

	return 0;
}

//! set home cell if no map is loaded about the home cell
int32_t Navigate2D::setHomeCell(int32_t floor_num, MazeCell::NavDir heading)
{
	m_home_floor_num = floor_num;
	m_chkpt_heading = heading;

	m_navigateMaps.getFloorMap(m_cur_floor_index)->allocateHomeCell("home");
	MazeCell *homecell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getHomeCell();
	homecell->setNavDirection(heading);
	m_home_cell_index = homecell->getCellNum();
	m_cur_cell_index = m_home_cell_index;
	return 0;
}

int32_t Navigate2D::setStairCell(int floor_num, MazeCell *stair_cell)
{
	if(stair_cell== NULL)
		return -1;
	MazeCell::NavDir heading;
	m_cur_floor_index = floor_num;

	heading = stair_cell->getNavDirection();
	int32_t indx = m_navigateMaps.getFloorMap(m_cur_floor_index)->allocateHomeCell("stair");
	MazeCell *staircell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(indx);
	staircell->setCellType(stair_cell->getCellType());
	staircell->setNavDirection(heading);
	staircell->setWallNorth(stair_cell->getWallNorth());
	staircell->setWallEast(stair_cell->getWallEast());
	staircell->setWallSouth(stair_cell->getWallSouth());
	staircell->setWallWest(stair_cell->getWallWest());
	m_navigateMaps.getFloorMap(m_cur_floor_index)->setStairCell(staircell);
	m_cur_cell_index = indx;
	return m_cur_cell_index;
}

//! from sensor detection
int32_t Navigate2D::configureCurCell(MazeCell *sensor_info)
{
	int32_t i;
	MazeCell *cur_cell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCurrentCell();

	cur_cell->setCheckPt(sensor_info->getCheckPt());
	cur_cell->setNonMovable(sensor_info->getNonMovable());
	cur_cell->setObstacle(sensor_info->getObstacle());
	cur_cell->setStairCell(sensor_info->getStairCell());
	cur_cell->setVictim(sensor_info->getVictim());
	cur_cell->setWallNorth(sensor_info->getWallNorth());
	cur_cell->setWallEast(sensor_info->getWallEast());
	cur_cell->setWallSouth(sensor_info->getWallSouth());
	cur_cell->setWallWest(sensor_info->getWallWest());
	cur_cell->setVisitStatus(MazeCell::Visited);
	if(cur_cell->getStairCell())
		m_navigateMaps.getFloorMap(m_cur_floor_index)->setStairCell(cur_cell);

	std::vector<int32_t> *vlist = m_navigateMaps.getFloorMap(m_cur_floor_index)->getVisitedList();
	bool matched = false;
	for(i = 0; i < vlist->size(); i++) {
		if(cur_cell->getCellNum() == (*vlist)[i])
			break;
	}
	if(!matched)
		(*vlist).push_back(cur_cell->getCellNum());

	std::vector<int32_t> *tbvlist = m_navigateMaps.getFloorMap(m_cur_floor_index)->getToBeVisitedList();
	matched = false;
	int32_t j;
	for(i = 0; i < tbvlist->size(); i++) {
		if(cur_cell->getCellNum() == (*tbvlist)[i]) {
			j = i;
			matched = true;
			break;
		}
	}
	if(matched)
		(*tbvlist).erase((*tbvlist).begin() + j);

	if(cur_cell->getStairCell()) 
	{
		if(m_cur_floor_index ==0) {
			int32_t temp_floor = 1;
			// this floor has not be explored
			if(m_navigateMaps.getFloorMap(temp_floor)->getCellSize() == 0) {
				m_cur_floor_index = 1;
				setStairCell(m_cur_floor_index, cur_cell);
			}
			else { // this floor has be explored
				if(m_home_floor_num != m_cur_floor_index)
					m_cur_floor_index = m_home_floor_num;
				m_cur_cell_index = m_navigateMaps.getFloorMap(m_cur_floor_index)->getStairCell()->getCellNum();
			}
		}
		else {
			int32_t temp_floor = 0;
			// this floor has not be explored
			if(m_navigateMaps.getFloorMap(temp_floor)->getCellSize() == 0) {
				m_cur_floor_index = 0;
				setStairCell(m_cur_floor_index, cur_cell);
			}
			else { // this floor has be explored
				if(m_home_floor_num != m_cur_floor_index)
					m_cur_floor_index = m_home_floor_num;
				m_cur_cell_index = m_navigateMaps.getFloorMap(m_cur_floor_index)->getStairCell()->getCellNum();
			}
		}
		
	}
	else {
		// update neighbor cell walls
		int32_t cellN_num = cur_cell->getNeighborCellNorth();
		if(m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(cellN_num) != NULL)
			m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(cellN_num)->setWallSouth(cur_cell->getWallNorth());

		int32_t cellE_num = cur_cell->getNeighborCellEast();
		if(m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(cellE_num) != NULL)
			m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(cellE_num)->setWallWest(cur_cell->getWallEast());

		int32_t cellS_num = cur_cell->getNeighborCellSouth();
		if(m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(cellS_num) != NULL)
			m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(cellS_num)->setWallNorth(cur_cell->getWallSouth());

		int32_t cellW_num = cur_cell->getNeighborCellWest();
		if(m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(cellW_num) != NULL)
			m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(cellW_num)->setWallEast(cur_cell->getWallWest());
	}
	return 0;
}

void Navigate2D::resetGraphMatrix()
{
	if(m_graph_matrix) {
		for(int32_t i = 0; i < m_graph_size; i++)
			delete m_graph_matrix[i];
		delete m_graph_matrix;
	}
	return;
}

void Navigate2D::allocateGraphMatrix(int32_t cellsize)
{
	int32_t i;
	if(m_graph_size != cellsize) {
		resetGraphMatrix();
		m_graph_size = cellsize;
		m_graph_matrix = (int32_t**) new int32_t*[m_graph_size];
		for(i = 0; i < m_graph_size; i++) {
			m_graph_matrix[i] = (int32_t*) new  int32_t[m_graph_size];
			memset(m_graph_matrix[i], 0, sizeof(int32_t)*m_graph_size);
		}
	}

	return;
}

int32_t Navigate2D::generateGraphMatrix()
{
	int32_t j;
	std::vector<MazeCell> *cell_list =  m_navigateMaps.getFloorMap(m_cur_floor_index)->getCellList();
	int32_t cellsize = (int32_t)cell_list->size();

	if(cellsize != m_graph_size)
		return -1;

	for(j = 0; j < cell_list->size(); j++) {
		if((*cell_list)[j].getNeighborCellNorth() >= 0) {
			int32_t cell_num = (*cell_list)[j].getNeighborCellNorth();
			m_graph_matrix[j][cell_num] = (*cell_list)[j].getWallNorth();
			m_graph_matrix[cell_num][j] = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell((*cell_list)[j].getNeighborCellNorth())->getWallSouth();
		}
		if((*cell_list)[j].getNeighborCellEast() >= 0) {
			int32_t cell_num = (*cell_list)[j].getNeighborCellEast();
			m_graph_matrix[j][cell_num] = (*cell_list)[j].getWallEast();
			m_graph_matrix[cell_num][j] = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell((*cell_list)[j].getNeighborCellEast())->getWallWest();
		}
		if((*cell_list)[j].getNeighborCellSouth() >= 0) {
			int32_t cell_num = (*cell_list)[j].getNeighborCellSouth();
			m_graph_matrix[j][cell_num] = (*cell_list)[j].getWallSouth();
			m_graph_matrix[cell_num][j] = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell((*cell_list)[j].getNeighborCellSouth())->getWallNorth();
		}
		if((*cell_list)[j].getNeighborCellWest() >= 0) {
			int32_t cell_num = (*cell_list)[j].getNeighborCellWest();
			m_graph_matrix[j][cell_num] = (*cell_list)[j].getWallWest();
			m_graph_matrix[cell_num][j] = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell((*cell_list)[j].getNeighborCellWest())->getWallEast();
		}
	}

	return 0;
}

std::string Navigate2D::getCurTime()
{
#ifdef WIN32
	SYSTEMTIME  system_time;
	GetLocalTime(&system_time);
	std::string cur_time = std::to_string(system_time.wYear) + "_" +
		std::to_string(system_time.wMonth) + "_" +
		std::to_string(system_time.wDay);
#else
	std::stringstream currentDateTime;
	// current date/time based on current system
	time_t ttNow = time(0);
	struct tm * now = localtime( & ttNow );
   cur_time = std::to_string(now->tm_year + 1900) + std::to_string(now->tm_mon + 1) + std::to_string(now->tm_mday);
#endif
	return cur_time;
}
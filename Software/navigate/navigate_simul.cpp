#include "navigate_simul.h"

NavigateSimul::NavigateSimul()
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
}

NavigateSimul::~NavigateSimul()
{
	m_newcell_list.clear();
	resetGraphMatrix();
}

// read in ground truth maps
int32_t NavigateSimul::readChkPtMaps(const char* out_dir, const char* filename)
{
	m_map_file = std::string(out_dir) + std::string(filename) + ".xml";
	if(m_gt_maps.readXmlMap(out_dir, filename) != 0)
		return -1;

	m_floors = m_gt_maps.getNumOfFloors();
	m_home_floor_num = m_gt_maps.getHomeFloorNum();

	for(int32_t i = 0; i < m_floors; i++)
		m_gt_maps.getFloorMap(i)->connectCells();

	return 0;
}

//! set home cell
int32_t NavigateSimul::setHomeCell(int32_t floor_num, MazeCell::NavDir heading)
{
	m_home_floor_num = floor_num;
	m_chkpt_heading = heading;

	// select a silver plate cell from the ground truth map to be the home cell
	std::vector<int32_t> *check_pt_list = m_gt_maps.getFloorMap(m_home_floor_num)->getCheckPtList();
	// pick a silver plate cell as the home cell
	int32_t index = rand() % (check_pt_list->size());

	m_navigateMaps.getFloorMap(m_home_floor_num)->allocateHomeCell("home");
	MazeCell *homecell = m_navigateMaps.getFloorMap(m_home_floor_num)->getHomeCell();
	homecell->setNavDirection(heading);
	homecell->setDebugLinkCell(m_gt_maps.getFloorMap(m_home_floor_num)->getCell((*check_pt_list)[index]));
	m_home_cell_index = homecell->getCellNum();
	m_cur_cell_index = m_home_cell_index;
	return 0;
}

int32_t NavigateSimul::setStairCell(int floor_num, MazeCell *stair_cell)
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
	staircell->setDebugLinkCell(m_gt_maps.getFloorMap(m_cur_floor_index)->getStairCell());
	staircell->setWallNorth(staircell->getDebugLinkCell()->getWallNorth());
	staircell->setWallEast(staircell->getDebugLinkCell()->getWallEast());
	staircell->setWallSouth(staircell->getDebugLinkCell()->getWallSouth());
	staircell->setWallWest(staircell->getDebugLinkCell()->getWallWest());
	m_navigateMaps.getFloorMap(m_cur_floor_index)->setStairCell(staircell);
	m_cur_cell_index = indx;
	return m_cur_cell_index;
}

//! display ground truth map 
bool NavigateSimul::displayGtMap(int32_t floor_num)
{
	int32_t status = m_gt_maps.displayPhysicalMap(floor_num);

	return (status == 0);
}

//! display current local map
bool NavigateSimul::displayLocalMap()
{
	int32_t status = m_navigateMaps.displayPhysicalMap(m_cur_floor_index);

	return (status == 0);
}

//! displat current local map with way points
bool NavigateSimul::displayRouteMap()
{
	int32_t status = m_navigateMaps.displayPhysicalMap(m_cur_floor_index, m_navigateMaps.getTracedRoute(m_cur_floor_index));

	return (status == 0);
}

//! from sensor detection
int32_t NavigateSimul::configureCurCell()
{
	int32_t i;
	MazeCell *cur_cell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCurrentCell();
	MazeCell *gt_cur_cell = (MazeCell*)cur_cell->getDebugLinkCell();

	cur_cell->setCheckPt(gt_cur_cell->getCheckPt());
	cur_cell->setNonMovable(gt_cur_cell->getNonMovable());
	cur_cell->setObstacle(gt_cur_cell->getObstacle());
	cur_cell->setStairCell(gt_cur_cell->getStairCell());
	cur_cell->setVictim(gt_cur_cell->getVictim());
	cur_cell->setVictimDirection(gt_cur_cell->getVictimDirection());
	cur_cell->setWallNorth(gt_cur_cell->getWallNorth());
	cur_cell->setWallEast(gt_cur_cell->getWallEast());
	cur_cell->setWallSouth(gt_cur_cell->getWallSouth());
	cur_cell->setWallWest(gt_cur_cell->getWallWest());
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

//! from sensor detection
int32_t NavigateSimul::detectLocalCells()
{
	int32_t i0, j0, i, j;
	int32_t cindx;
	MazeCell *cur_cell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(m_cur_cell_index);
	MazeCell::NavDir ndir = cur_cell->getNavDirection();
	cur_cell->getCellGrid(i0,j0);

	int32_t xo, yo, grid_w, grid_h; 
	MazeCell gt_cur_cell = *(cur_cell->getDebugLinkCell());
	gt_cur_cell.getCellGrid(xo,yo);
	grid_w = m_gt_maps.getFloorMap(m_cur_floor_index)->getGridHsize();
	grid_h = m_gt_maps.getFloorMap(m_cur_floor_index)->getGridVsize();

	m_newcell_list.clear();

	// north cells
	i = i0;
	j = j0;
	MazeCell *next_gt_cell = &gt_cur_cell;
	MazeCell *next_cell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(m_cur_cell_index);
	while(next_gt_cell!= NULL && (next_gt_cell->getWallNorth() == MazeCell::MOpen || 
		next_gt_cell->getWallNorth() == MazeCell::MBlack || next_gt_cell->getWallNorth() == MazeCell::MStair)) {
		next_gt_cell = m_gt_maps.getFloorMap(m_cur_floor_index)->getCell(next_gt_cell->getNeighborCellNorth());
		cindx = next_cell->getCellNum();
		next_cell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(next_cell->getNeighborCellNorth());
		j++;
		if((next_cell == NULL || next_cell->getVisitStatus() == MazeCell::NotFound) && next_gt_cell != NULL) {
			MazeCell *acell;
			int32_t index = m_navigateMaps.getFloorMap(m_cur_floor_index)->findNewCell(i,j);
			acell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(index);
			acell->setNonMovable(next_gt_cell->getNonMovable());
			acell->setCheckPt(next_gt_cell->getCheckPt());
			//acell->setCellType(next_gt_cell->getCellType()); // have not found yet
			acell->setObstacle(next_gt_cell->getObstacle());
			acell->setVictim(next_gt_cell->getVictim());
			acell->setVictimDirection(next_gt_cell->getVictimDirection());
			acell->setVisitStatus(MazeCell::TobeVisited);
			acell->setWallSouth(MazeCell::MOpen);
			acell->setWallNorth(next_gt_cell->getWallNorth());
			acell->setDebugLinkCell(next_gt_cell);
			next_cell = acell;
			m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(cindx)->updateNorthNeighbor(index);
			m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(index)->updateSouthNeighbor(cindx);
			m_newcell_list.push_back(acell->getCellNum());
		}
	}

	// east cells
	i = i0;
	j = j0;
	next_gt_cell = &gt_cur_cell;
	next_cell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(m_cur_cell_index);
	while(next_gt_cell!= NULL && (next_gt_cell->getWallEast() == MazeCell::MOpen || 
		next_gt_cell->getWallEast() == MazeCell::MBlack || next_gt_cell->getWallEast() == MazeCell::MStair)) {
		next_gt_cell = m_gt_maps.getFloorMap(m_cur_floor_index)->getCell(next_gt_cell->getNeighborCellEast());
		cindx = next_cell->getCellNum();
		next_cell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(next_cell->getNeighborCellEast());
		i++;
		if((next_cell == NULL || next_cell->getVisitStatus() == MazeCell::NotFound) && next_gt_cell != NULL) {
			MazeCell *acell;
			int32_t index = m_navigateMaps.getFloorMap(m_cur_floor_index)->findNewCell(i,j);
			acell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(index);
			acell->setNonMovable(next_gt_cell->getNonMovable());
			acell->setCheckPt(next_gt_cell->getCheckPt());
			//acell->setCellType(next_gt_cell->getCellType()); // have not found yet
			acell->setObstacle(next_gt_cell->getObstacle());
			acell->setVictim(next_gt_cell->getVictim());
			acell->setVictimDirection(next_gt_cell->getVictimDirection());
			acell->setVisitStatus(MazeCell::TobeVisited);
			acell->setWallWest(MazeCell::MOpen);
			acell->setWallEast(next_gt_cell->getWallEast());
			acell->setDebugLinkCell(next_gt_cell);
			next_cell = acell;
			m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(cindx)->updateEastNeighbor(index);
			m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(index)->updateWestNeighbor(cindx);
			m_newcell_list.push_back(acell->getCellNum());
		}
	}

	// south cells
	i = i0;
	j = j0;
	next_gt_cell = &gt_cur_cell;
	next_cell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(m_cur_cell_index);
	while(next_gt_cell!= NULL && (next_gt_cell->getWallSouth() == MazeCell::MOpen || 
		next_gt_cell->getWallSouth() == MazeCell::MBlack || next_gt_cell->getWallSouth() == MazeCell::MStair)) {
		next_gt_cell = m_gt_maps.getFloorMap(m_cur_floor_index)->getCell(next_gt_cell->getNeighborCellSouth());
		cindx = next_cell->getCellNum();
		next_cell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(next_cell->getNeighborCellSouth());
		j--;
		if((next_cell == NULL || next_cell->getVisitStatus() == MazeCell::NotFound) && next_gt_cell != NULL) {
			MazeCell *acell;
			int32_t index = m_navigateMaps.getFloorMap(m_cur_floor_index)->findNewCell(i,j);
			acell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(index);
			acell->setNonMovable(next_gt_cell->getNonMovable());
			acell->setCheckPt(next_gt_cell->getCheckPt());
			//acell->setCellType(next_gt_cell->getCellType()); // have not found yet
			acell->setObstacle(next_gt_cell->getObstacle());
			acell->setVictim(next_gt_cell->getVictim());
			acell->setVictimDirection(next_gt_cell->getVictimDirection());
			acell->setVisitStatus(MazeCell::TobeVisited);
			acell->setWallNorth(MazeCell::MOpen);
			acell->setWallSouth(next_gt_cell->getWallSouth());
			acell->setDebugLinkCell(next_gt_cell);
			next_cell = acell;
			m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(cindx)->updateSouthNeighbor(index);
			m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(index)->updateNorthNeighbor(cindx);
			m_newcell_list.push_back(acell->getCellNum());
		}
	}

	// west cells
	i = i0;
	j = j0;
	next_gt_cell = &gt_cur_cell;
	next_cell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(m_cur_cell_index);
	while(next_gt_cell!= NULL && (next_gt_cell->getWallWest() == MazeCell::MOpen || 
		next_gt_cell->getWallWest() == MazeCell::MBlack || next_gt_cell->getWallWest() == MazeCell::MStair)) {
		next_gt_cell = m_gt_maps.getFloorMap(m_cur_floor_index)->getCell(next_gt_cell->getNeighborCellWest());
		cindx = next_cell->getCellNum();
		next_cell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(next_cell->getNeighborCellWest());
		i--;
		if((next_cell == NULL || next_cell->getVisitStatus() == MazeCell::NotFound) && next_gt_cell != NULL) {
			MazeCell *acell;
			int32_t index = m_navigateMaps.getFloorMap(m_cur_floor_index)->findNewCell(i,j);
			acell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(index);
			acell->setNonMovable(next_gt_cell->getNonMovable());
			acell->setCheckPt(next_gt_cell->getCheckPt());
			//acell->setCellType(next_gt_cell->getCellType()); // have not found yet
			acell->setObstacle(next_gt_cell->getObstacle());
			acell->setVictim(next_gt_cell->getVictim());
			acell->setVictimDirection(next_gt_cell->getVictimDirection());
			acell->setVisitStatus(MazeCell::TobeVisited);
			acell->setWallEast(MazeCell::MOpen);
			acell->setWallWest(next_gt_cell->getWallWest());
			acell->setDebugLinkCell(next_gt_cell);
			next_cell = acell;
			m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(cindx)->updateWestNeighbor(index);
			m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(index)->updateEastNeighbor(cindx);
			m_newcell_list.push_back(acell->getCellNum());
		}
	}

	return 0;
}


//! update local map based on parsed sensor data
int32_t NavigateSimul::updateLocalMap()
{
	if(m_newcell_list.size() == 0)
		return -1;

	// update the group lists for each floor
	m_navigateMaps.getFloorMap(m_cur_floor_index)->connectCells();
	m_navigateMaps.getFloorMap(m_cur_floor_index)->updateCellArray();

	return 0;
}

//! navigation planning
int32_t NavigateSimul::navigatePlanning()
{
	int32_t i;
	int32_t cellsize = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCellSize();
	// do not need to go anywhere
	if(cellsize <=1)
		return -1;

	// compute shortest paths to a neighboring cell
	int32_t cur_cell_idex = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCurCellIndex();
	allocateGraphMatrix(cellsize);
	generateGraphMatrix();
	m_dijkstra.loadGraph(m_graph_matrix, m_graph_size);
	m_dijkstra.dijkstra(cur_cell_idex);
	m_dijkstra.printSolution();

	// from the legit cells, first remove visited nodes, and then pick the best one to be visited 
	std::vector<GreedyDijkstra::DistInfo>* dist_list = m_dijkstra.getDistList();
	for(i = 0; i < dist_list->size(); i++) {
		MazeCell *acell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell((*dist_list)[i].node_index);
		if((*dist_list)[i].node_index == 0) // home
			m_dijkstra.setHomePath((*dist_list)[i]);
		if(acell->getVisitStatus() == MazeCell::Visited) {
			dist_list->erase(dist_list->begin()+i);
			i--;
		}
	}
	for(i = 0; i < dist_list->size(); i++) {
		MazeCell *acell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell((*dist_list)[i].node_index);
		int32_t weights = acell->getWallNorth() + acell->getWallEast()+ acell->getWallSouth() + acell->getWallWest();
		(*dist_list)[i].weighted_dist = weights;
	}

	std::vector<GreedyDijkstra::DistInfo> *vlist = m_dijkstra.sortShortestPath();

	if(vlist != 0)
		m_next_cell = (*vlist)[0];
	else {
		std::vector<GreedyDijkstra::DistInfo>* lst = m_dijkstra.getDistList();
		m_next_cell = m_dijkstra.getHomePath();
	}
	return 0;
}

//! navigation
int32_t NavigateSimul::navigation2D()
{
	// get current robot position and orientation
	MazeCell::NavDir heading = MazeCell::navNorth;
	// based on way points to the next cell, compose a series of commands

	// send control commands and navigate one at a time

	// check if victim is available, black plate is available, silver plate is available, or obstacle is on the way

	// if they are available, change the commands accordingly to reach to the next cell

	// in simuation do nothing

	// set the destination cell to be the current cell
	int32_t waypts = (int32_t)m_next_cell.waypts.size();
	m_cur_cell_index = m_next_cell.waypts[0];
	m_navigateMaps.getTracedRoute(m_cur_floor_index)->push_back(m_next_cell);
	MazeCell *curcell = m_navigateMaps.getFloorMap(m_cur_floor_index)->getCell(m_cur_cell_index);
	curcell->setNavDirection(heading);
	curcell->setVisitStatus(MazeCell::Visited);
	m_navigateMaps.getFloorMap(m_cur_floor_index)->setCurCellIndex(m_cur_cell_index);
	return 0;
}

void NavigateSimul::resetGraphMatrix()
{
	if(m_graph_matrix) {
		for(int32_t i = 0; i < m_graph_size; i++)
			delete m_graph_matrix[i];
		delete m_graph_matrix;
	}
	return;
}

void NavigateSimul::allocateGraphMatrix(int32_t cellsize)
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

int32_t NavigateSimul::generateGraphMatrix()
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
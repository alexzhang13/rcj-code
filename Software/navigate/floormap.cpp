#include "cell.h"
#include "floormap.h"
#include <assert.h>
#include <string.h>


MazeFloorMap::MazeFloorMap()
{
	m_grid_wsize = 0;
	m_grid_hsize = 0;
	m_map_hsize = 0.0f;
	m_map_vsize = 0.0f;
	m_2dcell_array = NULL;
	m_stair_cell_index = -1;
	m_latest_checkpt_index = -1;
	m_cur_cell_index = -1;
	m_home_cell_index = -1;
	m_is_home = false;
	resetMap();
	m_initialized = false;
}


MazeFloorMap::~MazeFloorMap()
{
	resetMap();
}

void MazeFloorMap::resetMap()
{
	m_cell_list.clear();
	m_visited_list.clear();
	m_to_be_visited_list.clear();
	m_checkPtList.clear();
	m_unknown_list.clear();
	m_home_cell = NULL;
	m_latest_checkpt_cell = NULL;
	m_stair_cell = NULL;
	m_cur_cell = NULL;
	if(m_2dcell_array) {
		for(int32_t j = 0; j < m_grid_hsize; j++) {
			delete m_2dcell_array[j];
			m_2dcell_array[j] = NULL;
		}
		delete m_2dcell_array;
		m_2dcell_array = NULL;
	}
}


int32_t MazeFloorMap::allocateHomeCell(std::string basecell)
{
	std::string home = "home";
	MazeCell first_cell;
	int32_t sindex = 0;
	m_cur_cell_index = sindex;
	m_latest_checkpt_index = sindex;
	first_cell.setCellNum(sindex);
	first_cell.setCellGrid(0,0);
	first_cell.setCenterXY(0.0f, 0.0f);
	first_cell.setVisitStatus(MazeCell::Visited);
	first_cell.enableCellValid(true);

	// home
	if(strcmp(basecell.c_str(), home.c_str()) == 0) {
		first_cell.setHomeCell(true);
		first_cell.setCheckPt(true);
		m_cell_list.push_back(first_cell);
		assert(m_cell_list.size() ==1);
		m_home_cell = &(m_cell_list[0]);
		m_latest_checkpt_cell = &(m_cell_list[0]);
		m_checkPtList.push_back(m_cell_list[0].getCellNum());
	}
	// first cell after stair climbing 
	else {
		first_cell.setStairCell(true);
		m_cell_list.push_back(first_cell);
		assert(m_cell_list.size() ==1);
		m_stair_cell = &(m_cell_list[0]);
	}

	m_cur_cell = &(m_cell_list[0]);
	m_initialized = true; 
	m_grid_wsize = 1;
	m_grid_hsize = 1;
	m_map_hsize = m_cur_cell->getCellWidth();
	m_map_vsize = m_cur_cell->getCellWidth();

	m_visited_list.push_back(m_cell_list[0].getCellNum());
	return m_cur_cell_index;
}

int32_t MazeFloorMap::findNewCell(int32_t idx_x, int32_t idx_y)
{
	int32_t i;
	size_t cell_list_size = m_cell_list.size();
	int32_t cell_index = -1;
	assert(cell_list_size > 0);
	cell_index = (int32_t)cell_list_size;

	// allocate new cell
	MazeCell new_cell;
	new_cell.setCellWidth(m_cell_list[0].getCellWidth());
	new_cell.setCellNum(cell_index);
	new_cell.setCellGrid(idx_x, idx_y);
	new_cell.setCenterXY((float)idx_x*m_cell_list[0].getCellWidth(), (float)idx_y*m_cell_list[0].getCellWidth());
	new_cell.setVisitStatus(MazeCell::TobeVisited); // set to be visited
	new_cell.enableCellValid(true);

	m_cell_list.push_back(new_cell);
	m_to_be_visited_list.push_back(m_cell_list[cell_index].getCellNum());

	// find relationship of the new cell to the other cells
	int32_t x, y;
	for(i = 0; i < m_cell_list.size()-1; i++) {
		m_cell_list[i].getCellGrid(x,y);
		if(x == idx_x-1 && y == idx_y) {  
			m_cell_list[i].setWallEast(MazeCell::MOpen);
			m_cell_list[cell_index].setWallWest(MazeCell::MOpen);
			m_cell_list[i].updateEastNeighbor(new_cell.getCellNum());
			m_cell_list[cell_index].updateWestNeighbor(m_cell_list[i].getCellNum());
		}
		else if(x == idx_x+1 && y == idx_y) {
			m_cell_list[i].setWallWest(MazeCell::MOpen);
			m_cell_list[cell_index].setWallEast(MazeCell::MOpen);
			m_cell_list[i].updateWestNeighbor(new_cell.getCellNum());
			m_cell_list[cell_index].updateEastNeighbor(m_cell_list[i].getCellNum());
		}
		else if(x == idx_x && y == idx_y-1) {
			m_cell_list[i].setWallNorth(MazeCell::MOpen);
			m_cell_list[cell_index].setWallSouth(MazeCell::MOpen);
			m_cell_list[i].updateNorthNeighbor(new_cell.getCellNum());
			m_cell_list[cell_index].updateSouthNeighbor(m_cell_list[i].getCellNum());
		}
		else if(x == idx_x && y == idx_y+1) {
			m_cell_list[i].setWallSouth(MazeCell::MOpen);
			m_cell_list[cell_index].setWallNorth(MazeCell::MOpen);
			m_cell_list[i].updateSouthNeighbor(new_cell.getCellNum());
			m_cell_list[cell_index].updateNorthNeighbor(m_cell_list[i].getCellNum());
		}
	}

	return cell_index;
}

void MazeFloorMap::addNewCell(MazeCell &cell)
{
	m_cell_list.push_back(cell);
}

void MazeFloorMap::addNewCell(int32_t idx_x, int32_t idx_y, float cell_width)
{
	size_t cell_list_size = m_cell_list.size();
	int32_t cell_index = -1;
	cell_index = (int32_t)cell_list_size;

	// allocate new cell
	MazeCell new_cell;
	new_cell.setCellWidth(cell_width);
	new_cell.setCellNum(cell_index);
	new_cell.setCellGrid(idx_x, idx_y);
	new_cell.setCenterXY((float)idx_x*cell_width, (float)idx_y*cell_width);
	new_cell.setVisitStatus(MazeCell::NotFound); // set to be temoporarily not found 
	new_cell.enableCellValid(false);

	if(idx_x == m_grid_wsize -1) {
		new_cell.setWallEast(MazeCell::MWall);
	}
	else if(idx_x == 0) {
		new_cell.setWallWest(MazeCell::MWall);
	}

	if(idx_y == m_grid_hsize -1) {
		new_cell.setWallNorth(MazeCell::MWall);
	}
	else if(idx_y == 0) {
		new_cell.setWallSouth(MazeCell::MWall);
	}

	m_cell_list.push_back(new_cell);
	return;
}



int32_t MazeFloorMap::updateCell(int32_t idx_x, int32_t idx_y)
{
	int32_t cell_index = -1;
	assert(m_cell_list.size() > 0);


	return cell_index;
}

int32_t MazeFloorMap::connectCells()
{
	int32_t cellsize = getCellSize();
	int32_t i, j, m0, n0, m1, n1;

	// connect cells
	for(i = 0; i < cellsize; i++) {
		MazeCell *cell0 = getCell(i);
		cell0->getCellGrid(m0,n0);

		for(j = 0; j < cellsize; j++) {
			MazeCell *cell1 = getCell(j);
			cell1->getCellGrid(m1,n1);
			if(m0-1 == m1 && n0 == n1) {
				cell0->updateWestNeighbor(cell1->getCellNum());
				cell1->updateEastNeighbor(cell0->getCellNum());
			}
			else if(m0+1 == m1 && n0 == n1) {
				cell0->updateEastNeighbor(cell1->getCellNum());
				cell1->updateWestNeighbor(cell0->getCellNum());
			}
			if(m0 == m1 && n0-1 == n1) {
				cell0->updateSouthNeighbor(cell1->getCellNum());
				cell1->updateNorthNeighbor(cell0->getCellNum());
			}
			else if(m0 == m1 && n0+1 == n1) {
				cell0->updateNorthNeighbor(cell1->getCellNum());
				cell1->updateSouthNeighbor(cell0->getCellNum());
			}
		}
	}
	return 0;
}


int32_t MazeFloorMap::preallocCells(int32_t grid_width, int32_t grid_height)
{
	int32_t k;
	int32_t tnum = grid_width * grid_height;
	m_cell_list.clear();
	assert(m_2dcell_array == NULL);
	m_2dcell_array = new MazeCell **[grid_height];
	for(k = 0; k < grid_height; k++) 
		m_2dcell_array[k] = new MazeCell*[grid_width];

	m_grid_wsize = grid_width;
	m_grid_hsize = grid_height;

	for(int32_t j = 0; j < grid_height; j++) {
		k = grid_height-j-1;
		for(int32_t i = 0; i < grid_width; i++) {
			int32_t cell_index = k * grid_width + i;
			MazeCell new_cell;
			new_cell.setCellNum(cell_index);
			new_cell.setCellGrid(i, k);
			new_cell.setCenterXY((float)i*new_cell.getCellWidth(), (float)k*new_cell.getCellWidth());
			new_cell.setVisitStatus(MazeCell::NotFound); // set to be visited
			new_cell.enableCellValid(true);
			// check borders
			if(k == 0) {
				new_cell.setWallSouth(MazeCell::MWall);
			}
			if(k == grid_height -1) {
				new_cell.setWallNorth(MazeCell::MWall);
			}
			if(i ==0) {
				new_cell.setWallWest(MazeCell::MWall);
			}
			if(i == grid_width -1) {
				new_cell.setWallEast(MazeCell::MWall);
			}

			m_cell_list.push_back(new_cell);
			m_2dcell_array[j][i] = &new_cell;
		}
	}

	return tnum;
}

int32_t MazeFloorMap::updateCellArray()
{
	int32_t i, j, m, n;
	int32_t grid_w, grid_h;
	bool update_array_flag = false;
	bool matched;

	std::vector<MazeCell>::iterator it;
	for(i = 0; i < m_cell_list.size(); i++) {
		if(!m_cell_list[i].getValidCellFlag()) {
			m_cell_list.erase(m_cell_list.begin() + i);
			i--;
		}
	}

	// update group lists
	for(i = 0; i < m_cell_list.size(); i++) {
		if(m_cell_list[i].getCheckPt()) {
			matched = false;
			for(j = 0; j < m_checkPtList.size(); j++) {
				if(m_cell_list[i].getCellNum() == m_checkPtList[j]) {
					matched = true;
					break;
				}
			}
			if(!matched)
				m_checkPtList.push_back(m_cell_list[i].getCellNum());

		}
		if(m_cell_list[i].getVisitStatus() == MazeCell::Visited) {
			matched = false;
			for(j = 0; j < m_visited_list.size(); j++) {
				if(m_cell_list[i].getCellNum() == m_visited_list[j]) {
					matched = true;
					break;
				}
			}
			if(!matched)
				m_visited_list.push_back(m_cell_list[i].getCellNum());
		}
		else if(m_cell_list[i].getVisitStatus() ==MazeCell::TobeVisited) {
			matched = false;
			for(j = 0; j < m_to_be_visited_list.size(); j++) {
				if(m_cell_list[i].getCellNum() == m_to_be_visited_list[j]) {
					matched = true;
					break;
				}
			}
			if(!matched)
				m_to_be_visited_list.push_back(m_cell_list[i].getCellNum());
		}
		else if(m_cell_list[i].getVisitStatus() ==MazeCell::NotFound ||
			m_cell_list[i].getVisitStatus() ==MazeCell::Prohibited) {
				matched = false;
				for(j = 0; j < m_unknown_list.size(); j++) {
					if(m_cell_list[i].getCellNum() == m_unknown_list[j]) {
						matched = true;
						break;
					}
				}
				if(!matched)
					m_unknown_list.push_back(m_cell_list[i].getCellNum());
		}

		if(m_cell_list[i].getHomeCell())
			m_home_cell = &m_cell_list[i];
		if(m_cell_list[i].getStairCell())
			m_stair_cell = &m_cell_list[i];
	}

	int32_t min_i = 100*(m_grid_wsize + m_grid_hsize);
	int32_t min_j = min_i;
	int32_t max_i = -100*(m_grid_wsize + m_grid_hsize);
	int32_t max_j = max_i;
	int32_t cells_nums = getCellSize();
	if(cells_nums == 0)
		return 0;

	float cell_width = getCell(0)->getCellWidth();

	for(i = 0; i < cells_nums; i++) {
		getCell(i)->getCellGrid(m,n);
		min_i = std::min(m, min_i);
		min_j = std::min(n, min_j);
		max_i = std::max(m, max_i);
		max_j = std::max(n, max_j);
	}
	if(min_i ==0 && min_j ==0) {
		max_i = std::max(max_i, m_grid_wsize-1);
		max_j = std::max(max_j, m_grid_hsize-1);
	}
	else if(min_i < 0 && min_j == 0) {
		max_i = max_i - min_i;
		for(i = 0; i < cells_nums; i++) {
			getCell(i)->getCellGrid(m,n);
			getCell(i)->setCellGrid(m-min_i,n);
			getCell(i)->setCenterXY(float(m-min_i)*cell_width, (float)n*cell_width);
		}
	}
	else if(min_j < 0 && min_i == 0) {
		max_j = max_j - min_j;
		for(i = 0; i < cells_nums; i++) {
			getCell(i)->getCellGrid(m,n);
			getCell(i)->setCellGrid(m,n-min_j);
			getCell(i)->setCenterXY(float(m)*cell_width, (float)(n-min_j)*cell_width);
		}
	}
	else if(min_j < 0 && min_i < 0) {
		max_i = max_i - min_i;
		max_j = max_j - min_j;
		for(i = 0; i < cells_nums; i++) {
			getCell(i)->getCellGrid(m,n);
			getCell(i)->setCellGrid(m-min_i,n-min_j);
			getCell(i)->setCenterXY(float(m-min_i)*cell_width, (float)(n-min_j)*cell_width);
		}
	}

	grid_w = std::max(max_i+1, m_grid_wsize);
	grid_h = std::max(max_j+1, m_grid_hsize);

	if(grid_w != m_grid_wsize || grid_h != m_grid_hsize)
		update_array_flag = true;

	if(update_array_flag) {
		if(m_2dcell_array) {
			for(int32_t j = 0; j < m_grid_hsize; j++) 
				delete m_2dcell_array[j];
			delete m_2dcell_array;
		}
		m_grid_wsize = grid_w;
		m_grid_hsize = grid_h;
		m_2dcell_array = new MazeCell **[m_grid_hsize];
		for(j = 0; j < m_grid_hsize; j++) 
			m_2dcell_array[j] = new MazeCell *[m_grid_wsize];

		for(i = 0; i < cells_nums; i++) {
			getCell(i)->getCellGrid(m,n);
			m_2dcell_array[n][m] = getCell(i);
		}
	}

	setMapHsize((float)m_grid_wsize * cell_width);
	setMapVsize((float)m_grid_hsize * cell_width);

	// set current cell index to the checkpoint index
	m_cur_cell_index = m_latest_checkpt_index;

	return m_grid_wsize*m_grid_hsize;
}

#include "navigate2D.h"

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
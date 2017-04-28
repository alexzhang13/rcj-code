#include "navigate_main.h"
#include <greedy_dijkstra.h>
#include <cell.h>
#include <floormap.h>
#include <mazemap_gen.h>
#include <navigate_simul.h>
#include <navigate2D.h>


// simulation
int main(int argc, char **argv)
{
	//int ret = greedy_dikjstra_test();
	int ret = mapgen_test();
	//int ret = navigation_simul_test();
	//int ret = testMapLoad();
	return ret;
}


//http://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/
int greedy_dikjstra_test()
{
	const int32_t V = 9;
	/* non-negative graph example, 0 means no connection, >0 is valid weights*/
#if 0
	int32_t graph_example[9][9] = {
	{0, 4, 0, 0, 0, 0, 0, 8, 0},
	{4, 0, 8, 0, 0, 0, 0, 11, 0},
	{0, 8, 0, 7, 0, 4, 0, 0, 2},
	{0, 0, 7, 0, 9, 14, 0, 0, 0},
	{0, 0, 0, 9, 0, 10, 0, 0, 0},
	{0, 0, 4, 14, 10, 0, 2, 0, 0},
	{0, 0, 0, 0, 0, 2, 0, 1, 6},
	{8, 11, 0, 0, 0, 0, 1, 0, 7},
	{0, 0, 2, 0, 0, 0, 6, 7, 0}
	};
#else
	int32_t graph_example[9][9] = {
	{0, 1, 0, 0, 0, 0, 0, 1, 0},
	{1, 0, 1, 0, 0, 0, 0, 1, 0},
	{0, 1, 0, 1, 0, 1, 0, 0, 1},
	{0, 0, 1, 0, 100, 1, 0, 0, 0},
	{0, 0, 0, 100, 0, 100, 0, 0, 0},
	{0, 0, 1, 1, 100, 0, 1, 0, 0},
	{0, 0, 0, 0, 0, 1, 0, 1, 1},
	{1, 1, 0, 0, 0, 0, 1, 0, 1},
	{0, 0, 1, 0, 0, 0, 1, 1, 0}
	};
#endif

	int32_t *graph[V];
	for(int32_t i = 0; i < V; i++) {
		graph[i] = new int32_t[V];
		memcpy(graph[i], graph_example[i], sizeof(int32_t)*V);
	}

	GreedyDijkstra gd;
	gd.loadGraph(graph, 9);
	gd.dijkstra(1, 5);
	gd.printSolution();
	std::vector<GreedyDijkstra::DistInfo>* vlist = gd.sortShortestPath();

	for(int32_t i = 0; i < V; i++) {
		delete graph[i];
	}
	return 0;

}

int mapgen_test()
{
	const int32_t maze_width[2] = {5, 4};
	const int32_t maze_height[2] = {9, 8};
	const int32_t victims = 3;
	const int32_t nmovables = 2;
	const int32_t obstacles = 1;
	const int32_t checkpt = 3;
	const float walls = 0.2f;
	int32_t home_floor_num = 0;
	int32_t floors = 2;
	const char* out_dir = "C:/projects/StormingRobots2017/Data/map_data";
	const char* xmlname = "mazemap";
	const char* xmlname_new = "mazemap_new";
	// generate floor maps
	MazeMapGen mapgen;
	mapgen.updateParams(home_floor_num, floors);
	mapgen.initMap_floor0(maze_width[0], maze_height[0], checkpt, victims, nmovables, obstacles, walls);
	if(floors == 2)
		mapgen.initMap_floor1(maze_width[1], maze_height[1], checkpt, victims, nmovables, obstacles, walls);

	for(int32_t i = 0; i < floors; i++) {
		mapgen.generateSimulateMap(i);
		mapgen.displayPhysicalMap(i);
		mapgen.savePhysicalMap(out_dir, xmlname, i);
	}
	// write map to file
	mapgen.writeXmlMap(out_dir, xmlname);

	// read map file
	MazeMapGen mapgen_new;
	mapgen_new.readXmlMap(out_dir, xmlname);
	mapgen_new.writeXmlMap(out_dir, xmlname_new);
	for(int32_t i = 0; i < floors; i++) {
		mapgen_new.displayPhysicalMap(i);
		mapgen_new.savePhysicalMap(out_dir, xmlname_new, i);
	}

	return 0;
}

int navigation_simul_test()
{
	int32_t i;
	const char* in_dir = "C:/projects/StormingRobots2017/Data/map_data";
	const char* xmlname = "mazemap";
	int32_t home_floor_num = 0;
	MazeCell::NavDir heading = MazeCell::navNorth;

	NavigateSimul nav_simul;

	nav_simul.readChkPtMaps(in_dir, xmlname);
	nav_simul.displayGtMap(0);
	nav_simul.displayGtMap(1);

	// in real app, the information is derived from the input file
	nav_simul.setHomeCell(home_floor_num, heading);

	for(i = 0; i < 100; i++) {
		nav_simul.configureCurCell();
		nav_simul.detectLocalCells();
		nav_simul.updateLocalMap();
		//nav_simul.collectInfo(); // find victim or letter on the wall
		nav_simul.navigatePlanning();
		nav_simul.navigation2D();
		// go to the next cell
		// on the way, if see black plate or obstacle,
		// back to the previous way point
		// shall call configure cells
		nav_simul.displayRouteMap();
		_sleep(1000);
	}

	return 0;
}

int testMapLoad()
{
	int32_t i;
	const char* in_dir = "C:/projects/StormingRobots2017/Data/map_data";
	const char* xmlname = "mazemap_04272017";
	int32_t home_floor_num = 0;
	MazeCell::NavDir heading = MazeCell::navNorth;
	Navigate2D nav_rt;

	nav_rt.getCurTime();
	if(nav_rt.readChkPtMaps(in_dir, xmlname)!= 0) {
		nav_rt.setHomeCell(home_floor_num, heading);	
	}

	// configure wall and cell info
	// find victim or letter on the wall
	int32_t x, y;
	//nav_rt.(x,y);

	// parse sensor info
	MazeCell sensor_info; // filled in by sensor info
	MazeCell *cur_cell = nav_rt.getCurrentCell();
	int x, y;
	cur_cell->getCellGrid(x,y);
	nav_rt.configureCurCell(&sensor_info);
	// in cell operations: dropper, blinking, etc.

	// filled in by sensor info
	// neighbor cells update 
	// left 
	MazeCell cell_temp;
	std::vector<MazeCell> next_cell_list; 
	nav_rt.detectLocalCells(next_cell_list);

	// update local map
	nav_rt.updateLocalMap();

	// write local map to file - will move to another thead
	//nav_rt.writeMap(in_dir, xmlname);

	// line fitting to correct position and orientation
	//nav_rt.slam2d(); // will move to another thread

	// what to do next
	nav_rt.navigatePlanning();

	// move on to the next cell
	nav_rt.navigation2D();




	return 0;
}
/*
* The following algorithm is modified from 
* http://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/
*/

#ifndef _GREEDY_DIJKSTRA_H_
#define _GREEDY_DIJKSTRA_H_

#include "navigate_defs.h"
#include <string>
#include <vector>
#include <stdint.h>
#include "globals.h"

class NAVIGATE_EXPORT GreedyDijkstra {
public:
	typedef struct {
		int32_t dist;
		int32_t weighted_dist;
		int32_t node_index;
		std::vector<int32_t> waypts;
	} DistInfo;

	// constructor
	GreedyDijkstra();
	// destructor
	~GreedyDijkstra();

	int32_t loadGraph(int32_t **graph, int32_t nodesize);
	void dijkstra(int32_t from);
	std::vector<DistInfo>* sortShortestPath();

	void dijkstra(int32_t from, int32_t to);
	int32_t printSolution(int32_t from, std::vector<int32_t> dist, int32_t n, std::vector<int32_t> parent);
	void printSolution();
	void getShortPath(int32_t &dist, std::vector<int32_t> &path);

	inline int32_t **getGraph() { return m_graph; }
	inline std::vector<DistInfo>* getDistList() { return &m_dist_list;}

protected:
	static bool sortbyDistance(const DistInfo &node1, const DistInfo &node2);
	static bool sortbyWeightedDistance(const DistInfo &node1, const DistInfo &node2);
	int32_t minDistance(std::vector<int32_t> dist, std::vector<bool> sptSet);
	int32_t minDistance(int32_t dist[], bool sptSet[]);
	void resetGraph();
	void printPath(std::vector<int32_t> parent, int32_t j);

private:
	int32_t m_from;
	int32_t m_to;
	int32_t m_distance;
	std::vector<int32_t> m_path; 
	std::vector<DistInfo> m_short_paths;
	int32_t m_node_size;
	int32_t **m_graph;
	std::vector<int32_t> m_dist;  
	std::vector<bool> m_sptSet;
	std::vector<int32_t> m_parent;
	std::vector<DistInfo> m_dist_list;
};

#endif
//_GREEDY_DIJKSTRA_H_

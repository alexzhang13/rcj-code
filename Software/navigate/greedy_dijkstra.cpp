#include "greedy_dijkstra.h"
#include <stdio.h>
#include <limits.h>
#include <assert.h>
#include <array>
#include <vector>
#include <cstring>
#include <algorithm>

// constructor
GreedyDijkstra::GreedyDijkstra()
{
	m_node_size = 0;
	m_graph = NULL;
}

bool GreedyDijkstra::sortbyDistance(const DistInfo &node1, const DistInfo &node2)
{
	int32_t dist1 = node1.dist;
	int32_t dist2 = node2.dist;
	return (dist1 < dist2);
}

bool GreedyDijkstra::sortbyWeightedDistance(const DistInfo &node1, const DistInfo &node2)
{
	int32_t wdist1 = node1.weighted_dist;
	int32_t wdist2 = node2.weighted_dist;
	return (wdist1 > wdist2);
}

// destructor
GreedyDijkstra::~GreedyDijkstra()
{
	resetGraph();
}

void GreedyDijkstra::resetGraph()
{
	if(m_graph) {
		for(int i = 0; i < m_node_size; i++)
			delete m_graph[i];
		delete m_graph;
		m_graph = NULL;
	}
}

// load current graph
int32_t GreedyDijkstra::loadGraph(int32_t **graph, int32_t nodesize)
{
	int j;

	resetGraph();
	m_node_size = nodesize;

	assert(m_graph == NULL);
	m_graph = (int32_t **) new int32_t* [m_node_size];
	for(j =0; j < m_node_size; j++) {
		m_graph[j] = (int32_t *) new int32_t [m_node_size];
	}

	for(j = 0; j < m_node_size; j++) {
		memcpy(m_graph[j], graph[j], sizeof(int32_t)*m_node_size);
	}

	return 0;
}

int32_t GreedyDijkstra::minDistance(std::vector<int32_t> dist, std::vector<bool> sptSet)
{
	// Initialize min value
	int32_t min = MAZE_MAX_VAL, min_index;

	for (int32_t v = 0; v < m_node_size; v++)
		if (sptSet[v] == false && dist[v] <= min)
			min = dist[v], min_index = v;

	return min_index;
}


void GreedyDijkstra::printPath(std::vector<int32_t> parent, int32_t j)
{
	// Base Case : If j is source
	if (parent[j]==-1)
		return;

	printPath(parent, parent[j]);

	printf("%d ", j);
}


// A utility function to print the constructed distance
// array
int32_t GreedyDijkstra::printSolution(int32_t src, std::vector<int32_t> dist, int32_t n, std::vector<int32_t> parent)
{
    printf("Vertex\t  Distance\tPath");
    for (int32_t i = 0; i < m_node_size; i++)
    {
        printf("\n%d -> %d \t\t %d\t\t%d ", src, i, dist[i], src);
        printPath(parent, i);
	}
	return 0;
}

void GreedyDijkstra::dijkstra(int32_t from)
{
	int32_t i, count, u, v, val;
	m_dist_list.clear();
	m_dist.clear();  // The output array. dist[i] will hold
	// the shortest distance from src to i

	// sptSet[i] will true if vertex i is included / in shortest
	// path tree or shortest distance from src to i is finalized
	m_sptSet.clear();

	// Parent array to store shortest path tree
	m_parent.clear();
 
	 m_from = from;
    // Initialize all distances as INFINITE and stpSet[] as false
    for (i = 0; i < m_node_size; i++)
    {
        m_parent.push_back(-1);
		m_dist.push_back(MAZE_MAX_VAL);
		m_sptSet.push_back(false);
    }
 
    // Distance of source vertex from itself is always 0
    m_dist[from] = 0;
 
    // Find shortest path for all vertices
    for (count = 0; count < m_node_size-1; count++)
    {
        // Pick the minimum distance vertex from the set of
        // vertices not yet processed. u is always equal to src
        // in first iteration.
		u = minDistance(m_dist, m_sptSet);
 
        // Mark the picked vertex as processed
        m_sptSet[u] = true;
 
        // Update dist value of the adjacent vertices of the
        // picked vertex.
        for (v = 0; v < m_node_size; v++)
 
            // Update dist[v] only if is not in sptSet, there is
            // an edge from u to v, and total weight of path from
            // src to v through u is smaller than current value of
            // dist[v]
            if (!m_sptSet[v] && m_graph[u][v] &&
                m_dist[u] + m_graph[u][v] < m_dist[v])
            {
                m_parent[v]  = u;
                m_dist[v] = m_dist[u] + m_graph[u][v];
            }  
    }

	for(i = 0; i < m_node_size; i++) {
		m_path.clear();
		DistInfo dinfo;
		dinfo.node_index = i;
		dinfo.dist = m_dist[i];
		m_path.push_back(i);
		val = i;
		while(m_parent[val] != -1) {
			m_path.push_back(m_parent[val]);
			val = m_parent[val];
		}
		dinfo.waypts = m_path;
		m_dist_list.push_back(dinfo);
	}
	
	return;
}

void GreedyDijkstra::dijkstra(int32_t from, int32_t to)
{ 
	 m_from = from;
	 m_to = to;
 
	 dijkstra(from);

	m_distance = m_dist[m_to];

	int32_t val = m_to;
	m_path.push_back(val);
	while(m_parent[val] != -1) {
		m_path.push_back(m_parent[val]);
		val = m_parent[val];
	}
	return;
}

std::vector<GreedyDijkstra::DistInfo>* GreedyDijkstra::sortShortestPath()
{
	int32_t i;
	m_short_paths.clear();
	if(m_dist_list.size() < 1)
		return NULL;
	else {
		std::sort(m_dist_list.begin(), m_dist_list.end(), sortbyDistance);
	}

	int32_t dist0 = m_dist_list[0].dist;
	for(i = 0; i < m_dist_list.size(); i++) {
		if(m_dist_list[i].dist == dist0)
			m_short_paths.push_back(m_dist_list[i]);
	}

	if(m_short_paths.size() > 1) {
		std::sort(m_short_paths.begin(), m_short_paths.end(), sortbyWeightedDistance);

		int32_t wdist0 = m_short_paths[0].weighted_dist;
		for(i = 1; i < m_short_paths.size(); i++) {
			if(m_short_paths[i].weighted_dist != wdist0) {
				m_short_paths.erase(m_short_paths.begin() + i);
				i--;
			}
		}
	}

	return &m_short_paths;
}

void GreedyDijkstra::getShortPath(int32_t &dist, std::vector<int32_t> &path)
{
	dist = m_distance;
	path = m_path;
}

  // print the constructed distance array for all vertices
void GreedyDijkstra::printSolution()
{
    printf("Vertex\t  Distance\tPath");
    for (int32_t i = 0; i < m_node_size; i++)
    {
        printf("\n%d -> %d \t\t %d\t\t%d ", m_from, i, m_dist[i], m_from);
        printPath(m_parent, i);
    }
	return;
}

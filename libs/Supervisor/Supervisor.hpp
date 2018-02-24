/* A-star algorithm to search the optimal path planning
   this can be used as supervisor to train the network

*/

/* Here is defined the header file of A-star algorithm */

#ifndef ASTAR_H
#define ASTAR_H


#include <iostream>
#include <vector>

#define MAX_LENGTH 1000

using namespace std;

#include "Scenario.hpp"

using namespace PathPlanning;

namespace Supervisor
{


using uint = unsigned int;
using HeuristicFunction = std::function<uint(Pixel, Pixel)>;
using CoordinateList = std::vector<Pixel>;
using NodeSet = std::vector<Node*>;

class Astar
{	
 public:
	void setHeuristic(HeuristicFunction heuristic_);
	Astar(string& direction_mode);
	Astar(Scenario& scenario, string& direction_mode);

	void setHeuristic(string new_heuristic);
	
	float ComputeHeuristic(Pixel source, Pixel target);
	int getDirectionSize();
	float CalculateCost(CoordinateList& Path);
	
	int find_index(NodeSet& nodes, Node* curr);
	Node* findNodeOnList(NodeSet& nodes, Pixel& coordinates);
	int search_path(float& PathCost, CoordinateList& Path, int& index, double& elapsed_time, int& nodes_exp);
	int search_path(float& PathCost, CoordinateList& Path, Pixel& source_, Pixel& target_, int& nodes_exp);

	// function to search path in a scenario parameters not initially contained in .map.scen
	int search_extra_path(float& PathCost, CoordinateList& Path, Pixel& source_, Pixel& target_, int& nodex_exp);
	

	void releaseNodes(NodeSet& nodes);
protected:
	Scenario scenario;
	CoordinateList direction;
	string HeuristicMethod;
};

Pixel getDelta(Pixel source, Pixel target);

float euclidean(Pixel source, Pixel target);

void setDirections(vector<Pixel>& direction, string& mode);

bool OutOfBounds(Map& map, Pixel& next);

bool detectCollision(Node* newNode);

bool detectCollision(Node& newNode);

int find_node_coord(int map_x, int map_y, CoordinateList& Path);

float CalculateCost(CoordinateList& Path, string& direction_mode);

void PrintPath(CoordinateList& PathFound, Scenario& scenario, string& filename);

void PrintPath(CoordinateList& PathFound, Scenario& scenario, string& filename, char symbol);

void PrintPath(CoordinateList& PathFound, Map& map, string& filename, char symbol);

bool findPixel(int x, int y, LocalMap& occ);

}

#endif

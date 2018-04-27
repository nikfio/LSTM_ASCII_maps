/* Header file of RRT algorithm used here adapted to meet hypothesis
 * of our problem, environment is locally known (within robot sensor scope)
 * relative info robot - target are available like heuristic distance and 
 * relative angle
 */


#ifndef RRT_H
#define RRT_H


#include <vector>
#include <string>
#include <iostream>



#include "Scenario.hpp"
#include "Supervisor.hpp"

using namespace std;
using namespace PathPlanning;

// Uncomment to be more verbose
#define DEBUG_LEV_3

#define MAX_ITER 1000

#define PI 3.141592653

const int RADIUS_LEVEL_1 = 10;

const int RADIUS_LEVEL_2 = 25;

const int EXPLORED_TAIL = 4;

namespace Supervisor {

struct rrtNode {
    	vector<rrtNode *> children;
	rrtNode *parent;
	Pixel pix;
	char sym;
	bool blind;
	
	rrtNode();
	rrtNode(Pixel& pix);
	rrtNode(Pixel& new_pix, rrtNode * new_parent);
};
	 
using RRTList = vector<rrtNode>;

struct rrtMap
{
	vector< vector< rrtNode* > > coord;
	int width, height;
	
	rrtMap();
	~rrtMap();
	int getHeight();
	int getWidth();
};


class RRT 
{

public:
	RRT(Scenario& init_scen, string& direction_mode);

	rrtNode * RandomNode();
	rrtNode * RandomGoalBiasedNode();
	rrtNode * RandomGoalBiasedNode_low_bias();
	
	float distance(rrtNode * first, rrtNode * second);
	float nearest_dist_to_target(); 
	int getClosestDirection(double theta);
	
	bool isObstacle (Pixel& pix);

	rrtNode * Nearest(rrtNode * q_new);
	rrtNode * NewConfig(rrtNode * q_near, rrtNode * q_new);
	void TakeStep(rrtNode * new_config);
	rrtNode * CheckFreeRandomDir(rrtNode * new_conf, rrtNode * q_near);
	
	int search_path_normal(float& PathCost, CoordinateList& PathFound, int& index, double& elapsed_time);

	int search_path_goal_biased(float& PathCost, CoordinateList& PathFound, int& index, double& elapsed_time);

	int search_path_keep_free(float& PathCost, CoordinateList& PathFound, int& index, double& elapsed_time);

	void PrintTree(string& filename);

private:

	string direction_mode;
	CoordinateList direction;
	vector<double> steering_angles;
	rrtMap MapRRT;
	Map map;
	vector<rrtNode *> Tree;
	rrtNode * Root, *LastNode;
	Pixel source;
	Pixel target;
	vector<Scenario_par> param_vect;
	int step_size;
	string map_name;

	void To_rrtMap(Map& map);
	bool obstacleFreeEdge(Pixel& pix);
	void ToSteeringAngles();
};



} // namespace Supervisor


#endif

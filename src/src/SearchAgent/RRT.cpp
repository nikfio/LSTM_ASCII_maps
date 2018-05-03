/*
 * Implementation of functions defined in RRT.hpp, a version of RRT algorithm
 * for the benchmark (maps) used
 */



#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cfloat>

#include "Scenario.hpp"
#include "Supervisor.hpp"
#include "RRT.hpp"


using namespace std;
using namespace PathPlanning;
using namespace Supervisor;


namespace  Supervisor {


rrtNode::rrtNode() {
	
	pix.x = 0;
	pix.y = 0;
	sym = ' ';
	parent = nullptr;

};

rrtNode::rrtNode(Pixel& new_pix) {

	pix.x = new_pix.x;
	pix.y = new_pix.y;
	sym = ' ';
	parent = nullptr;

};


rrtNode::rrtNode(Pixel& new_pix, rrtNode * new_parent) {

	pix.x = new_pix.x;
	pix.y = new_pix.y;
	sym = ' ';
	parent = new_parent;

};

rrtMap::rrtMap() 
{
	for(int i = 0; i < MaxDim; i++) {
		coord.resize(MaxDim);
		for(int j = 0; j < MaxDim; j++) {
			coord[i].resize(MaxDim);
		}
	}
	
};

rrtMap::~rrtMap() {

	for(int i = 0; i < coord.size(); i++) {
		coord[i].erase(coord[i].begin(), coord[i].end() );
	}

};

int rrtMap::getHeight()
{
	return height;
};

int rrtMap::getWidth()
{
	return width;
};



void RRT::To_rrtMap(Map& map) {

	MapRRT.width = map.getWidth();
	MapRRT.height = map.getHeight();
	
	for(int x = 0; x < MapRRT.width; x++) {
		for(int y = 0; y < MapRRT.height; y++) {
			
			MapRRT.coord[x][y] = new rrtNode();
			MapRRT.coord[x][y]->pix.x = x;
			MapRRT.coord[x][y]->pix.y = y;
			MapRRT.coord[x][y]->sym = map.coord[x][y].sym;
			
		}

	}		

};

RRT::RRT(Scenario& init_scen, string& direction_mode) {

	setDirections(direction, direction_mode);	
	param_vect = init_scen.getParam_vect();
	direction_mode = direction_mode;
	map = init_scen.getMap();
	To_rrtMap(map);	
	map_name = init_scen.getMapName();
	ToSteeringAngles();

};


rrtNode * RRT::RandomNode() {

	Pixel new_pix;
	int prob = rand() % 100;

	if( prob < 20 )  {
		new_pix.x = target.x;
		new_pix.y = target.y;
	}
	else {
		new_pix.x = rand() % (MapRRT.width);
		new_pix.y = rand() % (MapRRT.height);
	}

	return MapRRT.coord[new_pix.x][new_pix.y];
	
};

float RRT::nearest_dist_to_target() {

	float dist;
	float min_distance = FLT_MAX;
	
	for(int i = 0; i < Tree.size(); i++) {
		dist = euclidean(target, Tree[i]->pix);
		if( dist < min_distance ) {
			min_distance = dist;
		}
	}
	
	return min_distance;


};

bool RRT::isObstacle (Pixel& pix) {

	if( MapRRT.coord[pix.x][pix.y]->sym == 'T' || MapRRT.coord[pix.x][pix.y]->sym == '@' ) 
		return true;
	else
		return false;

};


rrtNode * RRT::RandomGoalBiasedNode() {

	Pixel new_pix(0,0);
	int prob = rand() % 100;

	cout << target << endl;

	float dist_target = nearest_dist_to_target();

	if( prob < 20 || dist_target < RADIUS_LEVEL_1)  {
		new_pix.x = target.x;
		new_pix.y = target.y;
	}
	else if( prob < 40 ||  dist_target < RADIUS_LEVEL_2 ) {
		do {
		new_pix.x = target.x + ( rand() % RADIUS_LEVEL_1 );
		new_pix.y = target.y + ( rand() % RADIUS_LEVEL_1 );
		} while(!isObstacle(new_pix) );
	}
	else if( prob < 80 ) {
		do {
		new_pix.x = target.x + ( rand() % RADIUS_LEVEL_2 );
		new_pix.y = target.y + ( rand() % RADIUS_LEVEL_2 );
		} while(!isObstacle(new_pix) );
	}
	else {
		new_pix.x = rand() % (MapRRT.width);
		new_pix.y = rand() % (MapRRT.height);
	}

	return MapRRT.coord[new_pix.x][new_pix.y];
	
};



rrtNode * RRT::RandomGoalBiasedNode_low_bias() {

	Pixel new_pix(0,0);
	int prob = rand() % 100;

	float dist_target = nearest_dist_to_target();

	if( prob < 30 || dist_target < RADIUS_LEVEL_1)  {
		new_pix.x = target.x;
		new_pix.y = target.y;
	}
	else if( prob < 50 ||  dist_target < RADIUS_LEVEL_2 ) {
		do {
		new_pix.x = target.x + ( rand() % RADIUS_LEVEL_1 );
		new_pix.y = target.y + ( rand() % RADIUS_LEVEL_1 );
		} while(!isObstacle(new_pix));
	}
	else if( prob < 90 ) {
		do {
		new_pix.x = target.x + ( rand() % RADIUS_LEVEL_2 );
		new_pix.y = target.y + ( rand() % RADIUS_LEVEL_2 );
		} while(!isObstacle(new_pix));
	}
	else {
		new_pix.x = rand() % (MapRRT.width);
		new_pix.y = rand() % (MapRRT.height);
	}

	
	return MapRRT.coord[new_pix.x][new_pix.y];
	
};


float RRT::distance(rrtNode * first, rrtNode * second) {
	
	return euclidean(first->pix, second->pix);

};

rrtNode * RRT::Nearest(rrtNode * q_new) {

	float minimal_dist = FLT_MAX;
	rrtNode * nearest;
	int i;
	for(i = 0; i < Tree.size(); i++) {
		float dist = distance(q_new, Tree[i]);
		
		if( dist < minimal_dist ) {
			minimal_dist = dist;
			nearest = Tree[i];
		}
	
	}

	if( minimal_dist == FLT_MAX ) {
		cout << "Nearest: search failed!" << endl;
		exit(EXIT_FAILURE);
	}
	
	return nearest;

};


void RRT::ToSteeringAngles() {

	steering_angles.push_back( - PI/2 );       // direction.push_back(Pixel(0,1));
	steering_angles.push_back( 0 );    	   // direction.push_back(Pixel(1,0));
	steering_angles.push_back( PI/2 );	  // direction.push_back(Pixel(0,-1));
	steering_angles.push_back( 0 );         // direction.push_back(Pixel(-1,0));
	if(direction.size() == 4)
		return;

	steering_angles.push_back( 3*PI/4 );	 // direction.push_back(Pixel(-1,-1));
	steering_angles.push_back( - PI/4 );	 // direction.push_back(Pixel(1,1));
	steering_angles.push_back( - 3*PI/4 );	 // direction.push_back(Pixel(-1,1));
	steering_angles.push_back( PI/4 );	 // direction.push_back(Pixel(1,-1)); 
	
};

	
int RRT::getClosestDirection(double theta) {

	double min_dist_angle = DBL_MAX;
	int dir_index = -1;

	int i;
	for(i = 0; i < steering_angles.size(); i++) {
		double angle_dist = abs( theta - steering_angles[i] );
		cout << angle_dist << endl;
		if( angle_dist < min_dist_angle ) {
			min_dist_angle = angle_dist;
			dir_index = i;
		}

	}
	
	
	if( min_dist_angle == DBL_MAX ) {
		cout << "getClosestDirection: search failed!" << endl;
		exit(EXIT_FAILURE);
	}

	return dir_index;
	
};


rrtNode * RRT::NewConfig(rrtNode * q_near, rrtNode * q_new) {

	double theta = atan2( q_new->pix.y - q_near->pix.y, q_new->pix.x - q_near->pix.x);

	cout << "Theta: " << theta << endl;	
	
	int dir = getClosestDirection(theta);

	if( dir == 1 || dir == 3 ) {
		if( (q_new->pix.x - q_near->pix.x) > 0 ) 
			dir = 2;
		else if( (q_new->pix.x - q_near->pix.x) < 0 ) 
			dir = 4;
	}
	
	Pixel nextPix(q_near->pix + direction[dir]);
	cout << "Closest dir: " << dir << " Next: " << nextPix << endl;
	rrtNode *new_conf = MapRRT.coord[nextPix.x][nextPix.y];
	new_conf->parent = q_near;

	return new_conf;

};


void RRT::TakeStep(rrtNode * new_config) {

	if( new_config->parent->children.size() > 0 ) {
		new_config->children = new_config->parent->children;
	}
	
	new_config->children.push_back(new_config->parent);
	LastNode = new_config;

	Tree.push_back(new_config);

};

rrtNode * RRT::CheckFreeRandomDir(rrtNode * new_conf, rrtNode * q_near) {
	
	vector<rrtNode *> tail;
	tail.assign( q_near->children.end() - EXPLORED_TAIL, q_near->children.end() );
	CoordinateList available = direction;
	// prune directions occupied by obstacles
	for( int i = 0; i < available.size(); i++ ) {
		Pixel check = q_near->pix + available[i];
		if( MapRRT.coord[check.x][check.y]->sym != '.' )
			available.erase(available.begin() + i);	
	}
	
	if( available.size() == 1 ) {
		return nullptr;
	}

	// prune already explored space within the tail
	for( int i = 0; i < tail.size(); i++ ) {

		Pixel check = q_near->pix + available[i];
		for( int i = 0; i < available.size(); i++ ) { 
			if( MapRRT.coord[check.x][check.y]->pix == tail[i]->pix )
				available.erase(available.begin() + i);	
		}

	}
	
	if( available.empty() ) {
		return nullptr;
	}	
	else {
		int index = rand() % available.size();
		Pixel next = q_near->pix + available[index];
		rrtNode * FreeDir = MapRRT.coord[next.x][next.y]; 		
		return FreeDir;
	}
		



};


int RRT::search_path_normal(float& PathCost, CoordinateList& Path, int& index, double& elapsed_time) {


};


/* 
 * Implementation of RRT with goal biasing in point random generation and random directioning
 * if obstacle is encountered
 * this version checks for a random free direction also different from the previously explored (no turning back)
 * if there are not available unexplored free direction, no new node is added to the tree
 * COMMENT: forking from node distant to last node could be not feasible for a real robot
 */
int RRT::search_path_goal_biased(float& PathCost, CoordinateList& PathFound, int& index, double& elapsed_time) {

	Scenario_par temp_param = param_vect[index];

	source.x = temp_param.start_x;
	source.y = temp_param.start_y;
	target.x = temp_param.goal_x;
	target.y = temp_param.goal_y;
	
	rrtNode *current;
    	rrtNode *SourceNode = new rrtNode(source);
	rrtNode *TargetNode = new rrtNode(target);
		
	#ifdef DEBUG_LEV_3
        cout << "Agent RRT with goal-biased search called for map " << map_name << endl;
	cout << "Scenario's specifics: "<< endl << "Index " << index << endl;
	cout << "Goal :" << TargetNode->pix << endl;
	cout << "Source :" << SourceNode->pix << endl;
	cout << "Optimal length supposed " << temp_param.optimal_length << endl;
	#endif
	
	string test_RRT = "test_RRT" ;
	printMap_point(map, source, test_RRT, 'S');
	printMap_point(map, target, test_RRT, 'G');
   
 	Tree.push_back(SourceNode);

	float totalCost = 0;
	bool reached = false;

	int iter = 0;

	
	string test_TREE = "test_TREE";

	double start = getTime();
	while( iter < MAX_ITER ) {
		PrintTree(test_TREE);
		
		iter++;
		//Node *q_new = RandomNode()
		rrtNode *q_new = RandomGoalBiasedNode();
		cout << "Random point: " << q_new->pix << endl;
	
		printMap_point(map, q_new->pix, test_RRT, 'R');

		rrtNode *q_near = Nearest(q_new);
		
		cout << "Near point: " << q_near->pix << endl;
		rrtNode *new_conf = NewConfig(q_near, q_new);

		printMap_point(map, q_near->pix, test_RRT, 'W');
		
		getchar();
		if( !isObstacle(new_conf->pix) ) {
			cout << "q_near ADDED" << endl;
			TakeStep(new_conf);
		}
		else { 
			cout << "q_near is OBSTACLE" << endl;
			rrtNode *new_rand_conf = CheckFreeRandomDir(new_conf, q_near);
			if( new_rand_conf == nullptr ) {
				cout << "No dir possible, blind road" << endl;
				q_near->blind = true;
			}
			else {
				TakeStep(new_rand_conf);
				cout << "Taking a random free dir" << new_rand_conf->pix << endl;
				printMap_point(map, new_rand_conf->pix, test_RRT, 'F');
			}
			
		}
		
		if( LastNode->pix == target ) {
			cout << "TARGET" << endl;
			reached = true;
			break;
		}


	}

	rrtNode * nearest_to_target;
	if( reached ) {
		cout << "RRT: TARGET ACHIEVED" << endl;
		nearest_to_target = LastNode;
	}
	else {
		nearest_to_target = Nearest(TargetNode);
	}

	PathFound.push_back(nearest_to_target->pix);

	for(int i = (nearest_to_target->children.size() - 1); i >= 0; i--) {
		Pixel next = nearest_to_target->children[i]->pix;
		PathFound.push_back(next);
	}

	double end = getTime();

	elapsed_time = end - start;
	
	reverse(PathFound.begin(), PathFound.end());
	PathCost = CalculateCost(PathFound, direction_mode);
	

	if( reached ) 
		return 1;
	else
		return 0;

};


 

/* 
 * This version below of RRT is inspired by "Application of RRT-based local Path Planning Algorithm  
 * in Unknown Environment" by Yu Tian et al. (2007)
 * meets the hypothesis of unknown environment, main characteristic is that it keeps a goal biased random 
 * generated point until it's reached, if obstacle is encountered, free random direction is taken  
 * COMMENT: maybe more suitable for a real robot application
 */ 
int RRT::search_path_keep_free(float& PathCost, CoordinateList& PathFound, int& index, double& elapsed_time) {


	Scenario_par temp_param = param_vect[index];

	Pixel source, target;
	source.x = temp_param.start_x;
	source.y = temp_param.start_y;
	target.x = temp_param.goal_x;
	target.y = temp_param.goal_y;
	
	#ifdef DEBUG_LEV_3
        cout << "Agent RRT with keep-free search called for map " << map_name << endl;
	cout << "Scenario's specifics: "<< endl << "Index " << index << endl;
	cout << "Goal :" << target << endl;
	cout << "Source :" << source << endl;
	cout << "Optimal length suppixed " << temp_param.optimal_length << endl;
	#endif
		
	rrtNode *current;
    	rrtNode *SourceNode = new rrtNode(source);
	rrtNode *TargetNode = new rrtNode(target);
   
 	Tree.push_back(SourceNode);

	float totalCost = 0;
	bool reached = false;

	int iter = 0;
	double start = getTime();
	while( iter < MAX_ITER ) {
		
		iter++;
		//Node *q_new = RandomNode()
		rrtNode *q_new = RandomGoalBiasedNode_low_bias();

		//while( LastNode->pix == q_new->pix ) {
		while( LastNode->pix == q_new->pix ) {

			rrtNode *q_near = Nearest(q_new);
		
			rrtNode *new_conf = NewConfig(q_near, q_new);
			if( !isObstacle(new_conf->pix) ) {
				TakeStep(new_conf);
			}
			else { 
				rrtNode *new_rand_conf = CheckFreeRandomDir(new_conf, q_near);
				if( new_rand_conf == nullptr ) {
					q_near->blind = true;
					continue;
				}
				else {
					TakeStep(new_rand_conf);
				}
			}
			
			if( LastNode->pix == target ) {
				cout << "TARGET" << endl;
				reached = true;
				break;
			}
		
		}
	
		if( reached ) 
			break;
	
	}

	rrtNode * nearest_to_target;
	if( reached ) {
		nearest_to_target = LastNode;
	}
	else {
		nearest_to_target = Nearest(TargetNode);
	}

	PathFound.push_back(nearest_to_target->pix);

	for(int i = (nearest_to_target->children.size() - 1); i >= 0; i--) {
		Pixel next = nearest_to_target->children[i]->pix;
		PathFound.push_back(next);
	}
	
	double end = getTime();

	elapsed_time = end - start;

	reverse(PathFound.begin(), PathFound.end());
	PathCost = CalculateCost(PathFound, direction_mode);
	

	if( reached ) 
		return 1;
	else
		return 0;


}

void RRT::PrintTree(string& filename) {

	CoordinateList temp_tree;
	for(int i = 0; i < Tree.size(); i++) {
		temp_tree.push_back(Tree[i]->pix);
	}

	PrintPath(temp_tree, map, filename, 'T');

}
		



} // namespace Supervisor





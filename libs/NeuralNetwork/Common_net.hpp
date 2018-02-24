/* Functions in this header are meant to bind tie the neural network
 * library caffe to the Path Planning and Supervisor namespace 
 * functions:
 *	- prepare the input Datum for the network with current 
 *	  informations
 *
*/


#ifndef COMMON_NET_H
#define COMMON_NET_H

// C++ related
#include <string>
#include <vector>
#include <signal.h>
#include <ctype.h>

// Caffe related
#include <caffe/caffe.hpp>
#include <caffe/solver.hpp>
#include <caffe/util/io.hpp>

// project related
#include "Scenario.hpp"
#include "Supervisor.hpp"
#include "Build_set.hpp"

using std::vector;

using caffe::Datum;

using Supervisor::CoordinateList;

#define COLLISION_STATE 20

#define OUT_OF_BOUNDS_STATE 30

#define PATH_FINISHED_STATE 40


namespace NeuralNetwork
{

const int OCCUPANCY_WIDTH = 5;
const int OCCUPANCY_HEIGHT = 5;

const int DISTANCE_RANGE = 50;

const float PROXIM_WEIGHT = 2;

const float DIST_WEIGHT = 4;

const float ANGLE_WEIGHT = 1;

struct TrainVal_sets {
	setPaths TrainSet;
	setPaths ValidateSet;
	int TrainSize;
	int ValidateSize;
};

void AdjacentStatus(std::vector<float>& directionStatus, Pixel& current, Map& map, CoordinateList& direction);

float getRelativeAngle(Pixel& current, Pixel& target);

int getDirectionIndex(CoordinateList& direction, Pixel& nextPix);

vector<float> OccupancyData(Map& map, Pixel& current, Pixel& current_direction, int width, int height);

vector<float> LocalMapToData(LocalMap& occupancy);

void shutdown(int sign);

float getModDistance(Pixel& current, Pixel& target);

int getScenarioFromArchive(vector<Scenario> archive, string scenario_name); 

int getBlobSize(const float * array);

void PrintOccupancyGrid(Map& map, LocalMap& occupancy, string filename); 

/* Function updates the occupancy map with respect to current position of the agent
 * in order to give to the neural a network an always consistent vector representing this map
 * I developed this solution, surely not valid in terms of code compactness, but  
 * good in terms of runtime computation
 */
void OccupancyGrid(Map& map, LocalMap& occupancyMap, Pixel& current, Pixel& current_direction, int width, int height);

void DistanceMap(vector<float>& dist_state, Map& map, Pixel& current, CoordinateList& direction);

}


#endif

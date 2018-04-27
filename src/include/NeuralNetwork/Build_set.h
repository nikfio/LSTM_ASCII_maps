/* Header file to define functions to build sets from scenarios 
 * (and related maps) along with Astar results for them
 *

*/

#ifndef BUILD_SET_H
#define BUILD_SET_H


#include <vector>
#include <string>

#include "Scenario.hpp"
#include "Supervisor.hpp"

// Uncomment this below to be more verbose
//#define DEBUG_LEV_2


namespace NeuralNetwork {

const int RAND_SEED = 27;

using CoordinateList = std::vector<Pixel>;

struct setPaths {
	vector<Map> Maps;
	vector<int> map_index;
	vector<CoordinateList> OptimalPaths;
	vector<float> length;
	vector<int> expanded;
	double ElapsedTime;
};

struct BenchmarkSet{
	vector<Map> Maps;
	vector<int> map_index;
	vector<CoordinateList> OptimalPaths;
	vector<float> length;
	vector<double> solved_time;
	vector<int> expanded;
	int solved;
	int unsolved;
};

void build_set(ScenarioList& scen_list, setPaths& ResultPaths, int num_total_length, string& direction_mode);

void build_benchmarking_set(ScenarioList& scen_list, BenchmarkSet& FlySet, int set_total_paths, string& direction_mode);

} // namespace NeuralNetwork


#endif 

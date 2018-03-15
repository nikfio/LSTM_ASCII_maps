/* Function implementation related to 
 * header file build.set.hpp
 *
*/




#include <vector>
#include <iostream>
#include <cmath>

#include <glog/logging.h>

#include "boost/scoped_ptr.hpp"
#include "caffe/proto/caffe.pb.h"
#include "caffe/util/db.hpp"
#include "caffe/util/io.hpp"
#include "caffe/util/rng.hpp"
#include "caffe/util/format.hpp"


#include "Scenario.hpp"
#include "Supervisor.hpp"
#include "Build_set.hpp"
#include "Common_net.hpp"

using std::cout;
using namespace PathPlanning;
using namespace Supervisor;

DECLARE_int32(max_length);

/* keep in mind that scenarios are ordered in increasing complexity
 * from first to last available
 * thus this value below fix the minimal complexity that a path
 * has to have to be added to the set
*/
const int MINIMAL_COMPLEXITY = 0;

const int MINIMAL_LENGTH = 10;

const int MAXIMUM_COMPLEXITY = 750;

/* how much difference between cost path found by supervisor
 * and supposed optimal cost (length) 
 * if difference is < tolerance --> path found is added to the set
 * as verified supervised data
*/
int tolerance = 500;

namespace NeuralNetwork 
{


void build_set(ScenarioList& scen_list, setPaths& ResultPaths, int set_total_length, string& direction_mode) {
	
	LOG(INFO) << "Building set... SIZE: " << set_total_length << endl;


	const int MAXIMUM_LENGTH = FLAGS_max_length;

	srand(RAND_SEED);
	int amount=0;
	int par_index = 0;
	int current_length = 0;
	int total_length = 0;
	int nodes_exp;

	for(int i = 0; i < scen_list.scenario_archive.size(); i++ ) {
		ResultPaths.Maps.push_back(scen_list.scenario_archive[i].getMap());
	}

//	srand(time(NULL));

	double start = getTime();
	while( total_length < (set_total_length + amount) ) {
		
		 
		for(int j = 0; j<scen_list.getSize(); j++ % scen_list.getSize()) {
			
			Astar Agent(scen_list.scenario_archive[j], direction_mode);
			CoordinateList PathFound;
			float PathCost; 
			double elapsed_time;
			par_index = rand() % scen_list.scenario_archive[j].getSize();

			if( scen_list.scenario_archive[j].getOptimal(par_index) < MINIMAL_LENGTH ) {
				continue;
			}

			if( scen_list.scenario_archive[j].getOptimal(par_index) > MAXIMUM_LENGTH ) {
				continue;
			}

			if ( par_index < MINIMAL_COMPLEXITY ) {
				continue;
			}
	
			if( par_index > MAXIMUM_COMPLEXITY ) {
				continue;
			}
	
			Agent.search_path(PathCost, PathFound, par_index, elapsed_time, nodes_exp);

			amount++;
			current_length = PathFound.size();
			total_length += current_length;
			if( total_length > (set_total_length + amount) ) {
				int remainder = total_length - set_total_length - amount;
//				cout << "Remainder " << remainder << endl;
				PathFound.erase(PathFound.end() - remainder, PathFound.end());
				total_length += PathFound.size() - current_length;
			}						

			ResultPaths.OptimalPaths.push_back(PathFound);
			ResultPaths.map_index.push_back(j);
			ResultPaths.length.push_back(PathCost);
			ResultPaths.expanded.push_back(nodes_exp);
//			cout << "Progress: " << total_length << " Map index: " << j << " Total paths: " << amount << endl;
//			cout << "Paths loaded in current Set: " << amount << " scenario: " 
//			     << scen_list.scenario_archive[j].getScenarioName()
//			     << "  Index: " << par_index << endl;
//			string solved_path = "SET_path_" + std::to_string(set_total_length) + "-" + std::to_string(par_index);
//			PrintPath(PathFound, scen_list.scenario_archive[j].getMap(), solved_path, 'A');


		}
		
	}
	double end = getTime();
	ResultPaths.ElapsedTime = end - start;
	LOG(INFO) << "Finished Path set building: requested set size: " << set_total_length 
			<< " total paths: " << amount << " total length: " << total_length
			<< " time needed: "  << end - start << endl;
	
};


void build_benchmarking_set(ScenarioList& scen_list, BenchmarkSet& BenchmarkPaths, int set_total_paths, string& direction_mode) {

	LOG(INFO) << "Building Beanchmark set... SIZE: " << set_total_paths << endl;

	const int MAXIMUM_LENGTH = FLAGS_max_length;

	srand(RAND_SEED);
	int amount=0;
	int par_index = 0;
	int current_length = 0;
	int total_length = 0;

	for(int i = 0; i < scen_list.scenario_archive.size(); i++ ) {
		BenchmarkPaths.Maps.push_back(scen_list.scenario_archive[i].getMap());
	}
	
	while(amount < set_total_paths ) {

		 
		for(int j = 0; j<scen_list.getSize(); j++ % scen_list.getSize()) {
				
			par_index = rand() % scen_list.scenario_archive[j].getSize();

			if( scen_list.scenario_archive[j].getOptimal(par_index) < MINIMAL_LENGTH ) {
				continue;
			}

			if( par_index > MAXIMUM_COMPLEXITY ) {
				continue;
			}

			if( scen_list.scenario_archive[j].getOptimal(par_index) > MAXIMUM_LENGTH ) {
				continue;
			}

			if ( par_index < MINIMAL_COMPLEXITY ) {
				continue;
			}
	
			Astar Agent(scen_list.scenario_archive[j], direction_mode);
			float PathCost;
			CoordinateList PathFound;
			int result;
			double elapsed_time;
			int nodes_exp;
			result = Agent.search_path(PathCost, PathFound, par_index, elapsed_time, nodes_exp);
			
			BenchmarkPaths.solved_time.push_back(elapsed_time);

			if( result != 0 &&  PathCost < ( scen_list.scenario_archive[j].getOptimal(par_index) + 100 ) ) {
				BenchmarkPaths.solved++;
				//cout << "BenchmarkSet: solved:" << BenchmarkPaths.solved << endl;
			}

			if( PathCost > scen_list.scenario_archive[j].getOptimal(par_index) + 100 || result == 0 ) {
				BenchmarkPaths.unsolved++;
				//cout << "BenchmarkSet: unsolved:" << BenchmarkPaths.unsolved << endl;
			}

			amount++;
								
			BenchmarkPaths.OptimalPaths.push_back(PathFound);
			BenchmarkPaths.map_index.push_back(j);
			BenchmarkPaths.length.push_back(PathCost);
			BenchmarkPaths.expanded.push_back(nodes_exp);
//			cout << "Progress: " << total_length << " Map index: " << j << " Total paths: " << amount << endl;
//			cout << "Current step size: " << PathFound.size() << " optimal: " 
//			     << scen_list.scenario_archive[j].getOptimal(par_index)
//			     << "  Index: " << par_index << endl;
//			string solved_path = "BENCHMARK_path_" + std::to_string(amount);
//			PrintPath(PathFound, scen_list.scenario_archive[j].getMap(), solved_path, 'A');

			if(amount == set_total_paths) {
				break;
			}
			
				
		}			
			
		
	}
	
	LOG(INFO) << "Finished Benchmark Paths set building: total paths: " << amount << endl;

};
	

} // namespace NeuralNetwork
			













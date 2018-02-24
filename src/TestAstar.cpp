#include <cstdlib>
#include <string>
#include <iostream>
#include <vector>
#include <cmath>
#include <boost/filesystem.hpp>
#include <sstream>

#include "Scenario.hpp"
#include "Supervisor.hpp"

//#define DEBUG

using namespace std;
using namespace boost::filesystem;
using namespace PathPlanning;
using namespace Supervisor;

const int tolerance = 20;

int main(int argc,char* argv[]) {

if( argc != 4 ) {
	cout << "Not valid Scenarios folder" 
	     << "./TestAstar path-to-Scenarios-folder [num maps] " 
	     << "  [amount of paths per map]" << endl;
	exit(1);
}

int num_scen; 
int num_paths_per_scen; 
istringstream ss(argv[2]);

if (!(ss >> num_scen))
    cerr << "Invalid number 1 " << argv[1] << '\n';

istringstream ss1(argv[3]);

if (!(ss1 >> num_paths_per_scen))
    cerr << "Invalid number 2" << argv[3] << '\n';

path scen_folder = argv[1];

if( !boost::filesystem::exists(scen_folder) &&  !boost::filesystem::is_directory(scen_folder) ) {
	cout << "Not valid Scenarios folder" 
	     << "./TestAstar path-to-Scenarios-folder [num maps] " 
	     << "  [amount of paths per map]" << endl;
	exit(1);
}


#ifdef DEBUG
char confirm;
cout << "Test scenarios: " << num_scen << " num paths for each scenario map: " << num_paths_per_scen << endl
     << "Confirm? (y/n) ";
cin >> confirm;
     if( confirm == 'n' ) {
	cout << "Aborting by user" << endl;
	exit(1);
     }
#endif


vector<string> scen_paths;
directory_iterator end_iter;
string scen_ext = ".scen";
//string map_ext = ".map";

//cout << "Scenarios folder contain:" << endl;
for(directory_iterator itr(scen_folder); itr != directory_iterator(); ++itr) {
	if( is_regular_file(itr->status()) && itr->path().extension().string() == scen_ext ) {
			//cout << itr->path().string() << endl;
			scen_paths.push_back(itr->path().string());
	}

}


srand(27);
random_shuffle(scen_paths.begin(), scen_paths.end() );

vector<CoordinateList> PathsFound;
vector<float> CostsCalculated;

CostsCalculated.resize(num_scen);
PathsFound.resize(num_scen);

vector<string> TestPaths;

for( int i=0; i < num_scen; i++ ) {

	TestPaths.push_back( scen_paths[i] );

}


ScenarioList TestList(TestPaths, TestPaths.size(), 300 );

cout << "Initiating path finding..." << endl;

string directionMode = "diagonal";
int par_index;

int in_tolerance = 0;
int num_tests = 0;

int TEST_FUNCTION = 1;
int match = 0;

float cost_new = 0;
float cost = 0;
double elapsed_time; 
int nodes_exp;

for( int k = 0; k < num_scen; k++) {
	for(int i = 0; i < num_paths_per_scen; i++) {

		Astar Agent1(TestList.scenario_archive[k], directionMode);
		par_index = rand() % TestList.scenario_archive[k].getSize();
		CoordinateList PathFound;
		//cout << "par index: " << par_index << "  set size: " << TestList.scenario_archive[k].getSize() << endl;
		
		Agent1.search_path(cost, PathFound, par_index, elapsed_time, nodes_exp);
		float error = abs( cost - TestList.scenario_archive[k].getOptimal(par_index) );
		if ( error < tolerance ) {
			cout << "Cost calculated in tolerance limit, error:" << error << endl
			     << "Nodes expanded: " << nodes_exp << endl;	
			in_tolerance++;
		}
		else {
			cout << "Scenario: " << TestList.scenario_archive[k].getScenarioName() << endl
			     << "Predefined optimal length: " << TestList.scenario_archive[k].getOptimal(par_index)
			     << "   path cost exceed tolerance limit, error: " << error << "--> failure" << endl
			     << "Nodes expanded: " << nodes_exp << endl;	
		}
		num_tests++;

		if (TEST_FUNCTION) {
			Pixel current = TestList.scenario_archive[k].getSource(par_index); // PathFound[1];
			Pixel target = TestList.scenario_archive[k].getTarget(par_index);
			float newPathCost=0; CoordinateList newPathFound;
			Agent1.search_path(newPathCost, newPathFound, current, target, nodes_exp);
			cout << newPathCost << "     " << cost << endl;
			if( newPathCost == cost )
				match++;

			cout << "New fcn path cost: " << newPathCost << endl
			     << "Nodes expanded: " << nodes_exp << endl;	
			fflush(stdout);	
		}

	}

}


cout << "Supervisor Test: precision: " << in_tolerance << "/" << num_tests << "   tests met tolerance limit" << endl;

if( TEST_FUNCTION ) {
	cout << "Supervisor Test: matches with new function: " << match << "/" << num_tests << endl;
}



return 0;

}

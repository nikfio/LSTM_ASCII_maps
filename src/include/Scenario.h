/* 
	Header file for fucntions to parse a scenario file "name.map.scen" from MovingAI benchmarks,
	in order to load scenarios parameters and the related map
*/

#ifndef SCENARIO_H
#define SCENARIO_H

#include <string>
#include <vector>


using std::vector;
using std::string;

namespace GlobalPlanning 
{

// .map.scen single line parameters needed
struct Scenario_par {
	int bucket;
	std::pair<int,int> start;
	std::pair<int,int> goal;
	float optimal_length;
};

Scenario_par build_par(Pixel& current, Pixel& target, double optimal_len);

struct Map
{
	vector<char> coord;
	int width, height;
	
	Map();
	int getHeight();
	int getWidth();
};

// general map related functions
void printMap(Map& map, string& filename);

void mod_map_point(Map& map, std::pair<int,int>& point, char symbol);

void printMap_point(Map& map, std::pair<int,int>& point, string& filename, char symbol);


class Scenario 
{

public:
	Scenario();  

    // loads every scenario included in filename
	Scenario(string& filename);  

	// loads the first scenarios_number included in filename
	Scenario(string& filename, int scenarios_number); 

	~Scenario();
	
	string getScenarioName();
	string getMapName();
	int getSize();
	double getOptimal(int index);
	std::pair<int,int> getSource(int index);
	std::pair<int,int> getTarget(int index);
	Scenario_par& getScenario_par(int index);
	vector<Scenario_par>& getParam_vect();
	Map& getMap();	

	// from text file to Map struct conversion
	int MapToNodes();

	void printScenario(int index);
	void printMap(string& new_file);	
	void printMap_point(Pixel& point, string& filename, char symbol);
	
private:
	string name;
	string scen_name;
	string map_name;
	int scenarios_amount;
	Map scenario_map;
	vector<Scenario_par>  scenario_vect;

}; // end of scenario class


class ScenarioList
{		

	public:
		// Loads every Scenario related to names in string_list and for each one
		// every scenario parameters available is loaded
		ScenarioList(std::vector<string>& string_list); 
		
		// Loads the first tot_scenarios in string_list, and for each one
		// every scenario parameters available is loaded
		ScenarioList(std::vector<string>& string_list, int tot_scenario_set);
		
		// Loads the first tot_scenarios in string_list, and for each one
		// every scenario parameters available is loaded
		ScenarioList(std::vector<string>& string_list, int tot_scenario_set, int scenarios_number);
		
		int getSize();
		
		// ~ScenarioList(); // No need since vector class manages destruction work
	
		std::vector<Scenario> scenario_archive;
		
};

void BuildPar(Scenario_par& newPar, Pixel& current, Pixel& target);

double getTime();

} // GlobalPlanning namespace

#endif


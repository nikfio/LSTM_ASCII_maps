/* 
	Header file for fucntions to parse a scenario file "name.map.scen" from MovingAI benchmarks,
	in order to load scenarios parameters and the related map
*/

#ifndef SCENARIO_H
#define SCENARIO_H

#include <string>
#include <vector>


namespace GlobalPlanning 
{

using pixel = std::<int,int>;

// .map.scen single scenario parameters 
struct Scenario_par {
	pixel start;
	pixel goal;
	float optimal;
};

/*
	base class to handle the map
*/
class Map
{
	public:	
	Map();
	
	// from text file to Map struct conversionh
	void TextToMap(string& map_name);
	inline int getHeight() { return height; };
	inline int getWidth() { return width; };
	
	void printMap(string& filename);

	void mod_map_point(const pixel& point, const char symbol);

	void printMap_point(const pixel& point, 
						const string& filename,
						const char symbol,
						const bool mod);

	protected:
	std::string coord;
	int width, height;

};


/*
	class to handle map and related list of scenarios
*/
class Scenario
{

	public:
	// empty 
	Scenario();  

    // loads every scenario listed in filename
	Scenario(std::string& filename);  

	// loads the first scenarios_number listed in filename
	Scenario(std::string& filename, int scenarios_number)

	~Scenario();
	
	inline std::string getScenarioName() { return scen_name; }
	inline std::string getMapName() { return map_name; }
	inline Map& getMap() { return scenario_map; }
	inline int getSize() { return scenario_vect.size(); }
	inline float getOptimal(const uint index) { return scenario_vect[index].optimal; }
	inline pixel getSource(const int index) { return scenario_vect[index].start; }
	inline pixel getTarget(const int index) { return scenario_vect[index].goal; }
	inline Scenario_par& getScenario_par(const int index) { return scenario_vect[index]; }

	private:
	std::string scen_name;
	std::string map_name;
	Map scenario_map;
	std::vector<Scenario_par> scenario_vect;

}; 


class ScenarioList
{		

	public:
	// Loads every Scenario related to names in string_list and for each one
	// every scenario parameters available is loaded
	ScenarioList(std::vector<string>& string_list); 
		
	// Loads the first tot_scenarios in string_list, and for each one
	// every scenario parameters available is loaded
	ScenarioList(std::vector<string>& string_list, int scenarios_number);
		
	inline int getSize() { return scenario_archive.size(); }
		
	private
	std::vector<Scenario> scenario_archive;
		
};

double getTime();

} // GlobalPlanning namespace

#endif


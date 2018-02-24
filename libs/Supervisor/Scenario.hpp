/* Class to parse a scenario file "name.map.scen" from MovingAI benchmarks,
   in order to load scenarios parameters


*/

/* Header file for the class Scenario */


#ifndef SCENARIO_H
#define SCENARIO_H

#include <string>
#include <vector>
#include <functional>
#include <set>

// Uncomment this below to be more verbose
//#define DEBUG_LEV_1


// maximum extensions for maps dimension
const int MaxDim = 2000;

using namespace std;


namespace PathPlanning 
{

struct Pixel {

	int x, y;
	
	Pixel();
	Pixel(const Pixel& new_pix);
	Pixel(int x, int y);
	Pixel(Pixel& pix, int add_x, int add_y);
	void add(int add_x, int add_y);

	bool operator == (const Pixel& coord);
	bool operator != (const Pixel& coord);
	const Pixel operator + (const Pixel& second);
	const Pixel operator - (const Pixel& second);
	friend ostream& operator << (ostream& outputStream, const Pixel& pix);
};

struct Scenario_par {
	int bucket;
	string filename;
	int width;
	int height;
	int start_x;
	int start_y;
	int goal_x;
	int goal_y;
	double optimal_length;
};

Scenario_par build_par(Pixel& current, Pixel& target, double optimal_len);

struct Node 
{
	double G, H;
	Pixel pix;
	char sym;	
	Node *parent;

	double getAstarScore();
	Node();	

	Node(Pixel pix);
	Node(Pixel coord_, Node* parent_);
	const Node operator = (const Node& equival);
	bool operator == (const Node& curr);
	
};

struct PixelMap
{
	Pixel pix;
	char sym;

	PixelMap(int x, int y, char symbol);
	PixelMap(Pixel& newPix, char symbol);
};

using LocalMap = std::vector<PixelMap>;

struct Map
{
	vector< vector<Node> > coord;
	int width, height;
	
	Map();
	int getHeight();
	int getWidth();
};


class Scenario 
{

public:
	Scenario();  // empty, not useful
	Scenario(string& filename);  // loads every scenario included in filename
	Scenario(string& filename, int scenarios_number); // loads the first scenarios_number  
							  // included in filename
	
	string getScenarioName();
	string getMapName();
	int getSize();
	double getOptimal(int index);
	Pixel getSource(int index);
	Pixel getTarget(int index);
	Scenario_par& getScenario_par(int index);
	vector<Scenario_par>& getParam_vect();
	Map& getMap();	
	
	//LoadNewScenario(ifstream& filename);
	//LoadNewScenario_par(int amount);


	int MapToNodes();
	void printScenario(int index);
	void printMap();
	void printMap(string& new_file);	
	void printMap_point(Pixel& point, string& filename, char symbol);
	void deleteMap();

	//~Scenario();
	
	vector<Scenario_par>  scenario_vect;
private:
	string name;
	string scen_name;
	string map_name;
	int scenarios_amount;
	Map scenario_map;

}; // end of scenario class

// external utilities
void printMap(Map& map, string& filename);

void printMap_point(Scenario& scen, Pixel& point, string& filename, char symbol);

void printMap_point(Map& map, Pixel& point, string& filename, char symbol);

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

} // PathPlanning namespace

#endif


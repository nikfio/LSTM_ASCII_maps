/* Class to parse a scenario file "name.map.scen" from MovingAI benchmarks,
   in order to load scenarios parameters


*/

/* here are implemented member functions defined in "scenario.h" */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <sys/time.h>
#include <ctype.h>

#include <boost/filesystem.hpp>

#include "Scenario.hpp"


using namespace std;
using namespace boost::filesystem;


namespace PathPlanning {

Pixel::Pixel() {

	x=0; y=0;
};

bool Pixel::operator == (const Pixel& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
};

bool Pixel::operator != (const Pixel& coordinates_)
{
    if( x == coordinates_.x && y == coordinates_.y) 
	return false;
    else
	return true;
    
};


const Pixel Pixel::operator + (const Pixel& second)
{   
    Pixel add;
    add.x = x + second.x; add.y = y + second.y;
    return add;
};

const Pixel Pixel::operator - (const Pixel& second)
{   
    Pixel sub;
    sub.x = x - second.x; sub.y = y - second.y;
    return sub;
};



Pixel::Pixel(const Pixel& new_pix) 
{
	x = new_pix.x; 
	y = new_pix.y;
};

Pixel::Pixel(int new_x,int new_y) 
{
	x = new_x; 
	y = new_y;
};

Pixel::Pixel(Pixel& pix, int add_x, int add_y) {

	x = pix.x + add_x;
	y = pix.y + add_y;

};

void Pixel::add(int add_x, int add_y) {
	
	x += add_x;
	y += add_y;
};

ostream& operator << (ostream& outputStream, const Pixel& pix) {
	
	outputStream << "(" << pix.x << "," << pix.y << ")";

};

Node::Node() 
{
	pix.x = 0;
	pix.y = 0;
	G = 0; H = 0;
	parent = nullptr;
};


Node::Node(Pixel new_pix)
{
	pix.x = new_pix.x;
	pix.y = new_pix.y;
	parent = nullptr;
	G = 0; H = 0;
};

Node::Node(Pixel coordinates_, Node* parent_)
{
    parent = parent_;
    pix = coordinates_;
    G = 0; H = 0;
   
};

double Node::getAstarScore()
{
    return G + H;
};

const Node Node::operator = (const Node& equal)
{
    pix = equal.pix;
    G = equal.G;
    H = equal.H;
    sym = equal.sym;
    parent = equal.parent;
};

bool Node::operator == (const Node& curr)
{
	if( pix==curr.pix && G==curr.G && H==curr.H && parent==curr.parent)
		return true;
	else
		return false;

}

PixelMap::PixelMap(int x, int y, char symbol) {

	pix.x = x;
	pix.y = y;
	sym = symbol;

};

PixelMap::PixelMap(Pixel& newPix, char symbol) {

	pix.x = newPix.x;
	pix.y = newPix.y;
	sym = symbol;

};


Map::Map() : coord(MaxDim, vector<Node>(MaxDim) )
{

};

int Map::getHeight()
{
	return height;
};

int Map::getWidth()
{
	return width;
};


Scenario::Scenario() {

};

//const Scenario Scenario::operator = (const Scenario& newScen) {
//	
//	name = newScen.name;
//	scen_name = newScen.scen_name;
//	map_name = newScen.map_name;
//	scenarios_amount = newScen.scenarios_amount;
//	scenario_map = newScen.scenario_map;
//	scenario_vect = newScen.scenario_vect;

//}


Scenario::Scenario(string& filename) {
	
	std::ifstream temp;
	temp.open(filename.c_str());

	if( temp.fail() ) {
		cout << "Scenario file opening failed.\n";
		exit(1);
	}
	
	name = filename;
	temp.close();
	
	path p(name);
	string scen_folder = p.parent_path().string();
        scen_name = p.filename().string();
	string map_folder = scen_folder.substr(0, scen_folder.length() - 9) + "Maps/";
	map_name = map_folder + p.stem().string();

//	cout << "Map: " << map_name << endl;

	temp.open(map_name.c_str());
	if( temp.fail() ) {
		cout << "Map file opening failed.\n";
		exit(1);
	}
	
	string object_analyzed;
	getline(temp, object_analyzed);
	temp >> object_analyzed >> scenario_map.height;
	temp >> object_analyzed >> scenario_map.width;
	temp >> object_analyzed; 
	#ifdef DEBUG_LEV_1
	cout << "analyzing a " << object_analyzed << endl;
	#endif
	temp.close();

	if ( !MapToNodes() ) {
		cerr << "Error loading the map\n";
		exit(1);
	}

	temp.open(filename.c_str());

	if( temp.fail() ) {
		cout << "Scenario file opening failed.\n";
		exit(1);
	}

	
	temp.seekg(0, temp.beg);
	
	int i; char next; string map_version; 
	Scenario_par temp_par;

	getline(temp, map_version);
	
	#ifdef DEBUG_LEV_1
	cout << "Reading scenario: " << name << "  version: " << map_version << '\n';
	#endif	

	int line = 0;
	string parameters_line; 
	while( !temp.eof() ) {

	getline(temp, parameters_line);
	stringstream ss;
	ss << parameters_line;
	ss >> temp_par.bucket >> temp_par.filename >> temp_par.width >> temp_par.height 
	   >> temp_par.start_x >>temp_par.start_y >> temp_par.goal_x >> temp_par.goal_y 
	   >> temp_par.optimal_length;

	scenario_vect.push_back(temp_par);

	line++;
	}
	
	scenarios_amount = line;
	
	temp.close();

};

Scenario::Scenario(string& filename, int scenarios_number) {
	
	std::ifstream temp;
	temp.open(filename.c_str());

	if( temp.fail() ) {
		cout << "Scenario file opening failed.\n";
		exit(1);
	}
	
	name = filename;
	temp.close();

	path p(name);
	string scen_folder = p.parent_path().string();
        scen_name = p.filename().string();	
	string map_folder = scen_folder.substr(0, scen_folder.length() - 9) + "Maps/";
        map_name = map_folder + p.stem().string();

//	cout << "Map: " << map_name << endl;

	temp.open(map_name.c_str());
	if( temp.fail() ) {
		cout << "Map file opening failed.\n";
		exit(1);
	}

	string object_analyzed;
	getline(temp, object_analyzed);
	temp >> object_analyzed >> scenario_map.height;
	temp >> object_analyzed >> scenario_map.width;
	
	temp >> object_analyzed; 
	
	
	temp.close();

	if ( !MapToNodes() ) {
		cerr << "Error loading the map\n";
		exit(1);
	}
	
	temp.open(filename.c_str());

	if( temp.fail() ) {
		cout << "Scenario file opening failed.\n";
		exit(1);
	}

	temp.seekg(0, temp.beg);
	
	int i; char next; string map_version; 
	Scenario_par temp_par;

	getline(temp, map_version);
	#ifdef DEBUG_LEV_1
	cout << "Reading scenario: " << name << "  version: " << map_version << endl;
	#endif
	int line = 0;
	string parameters_line;

	while( line < scenarios_number && !temp.eof() ) {

	getline(temp, parameters_line);
	stringstream ss;
	ss << parameters_line;
	ss >> temp_par.bucket >> temp_par.filename >> temp_par.width >> temp_par.height 
	   >> temp_par.start_x >>temp_par.start_y >> temp_par.goal_x >> temp_par.goal_y 
	   >> temp_par.optimal_length;

	scenario_vect.push_back(temp_par);

	line++;
	}
	

	scenarios_amount = line;
	temp.close();

};


void Scenario::printScenario(int index) {
	
	vector<Scenario_par>::iterator it;
	int i;
	for(it = scenario_vect.begin(); it != scenario_vect.end(); ++it) {
		if( i == index ) {
			cout << "Bucket " << it->bucket
			     << setw(5) << "    Map name " << it->filename
			     << setw(5) << "    Width " << it->width
			     << setw(5) << "    Height " << it->height 
			     << setw(5) << "    Start x " << it->start_x 
			     << setw(5) << "    Start y " << it->start_y
			     << setw(5) << "    Goal x" << it->goal_x
			     << setw(5) << "    Goal y" << it->goal_y
			     << setw(5) << "    Optimal length " << it->optimal_length << endl;
		}
	}

};

int Scenario::MapToNodes() {
	
	std::ifstream temp;
	temp.open(map_name.c_str());
	if( temp.fail() ) {
		cout << "MapToNodes: Map file opening failed.\n";
		exit(1);
	}
	
	#ifdef DEBUG_LEV_1
	cout << "Opening "<< map_name << endl;
	#endif	
	
	temp.seekg(0, temp.beg);
	int line = 0; 
	char next;
 	string object_analyzed;

	while(line < 4 ) {
		getline(temp, object_analyzed);
		#ifdef DEBUG_LEV_1
		cout << line << "  " << object_analyzed << endl;
		#endif
		line++;
	}
	
	
	int x,y;
	string temp_line;
	const char * linecopy = new char[scenario_map.width];
	
	for(y = 0; y<scenario_map.height; y++) {
		getline(temp, temp_line);
		linecopy = temp_line.c_str();
		for(x=0; x<scenario_map.width; x++) {
			scenario_map.coord[x][y].sym = linecopy[x];
			scenario_map.coord[x][y].pix.x = x;
			scenario_map.coord[x][y].pix.y = y;
		}
		
	}
	
	temp.close();
	
	return 1;

};


void BuildPar(Scenario_par& newPar, Pixel& current, Pixel& target) {
	
	newPar.start_x = current.x;
	newPar.start_y = current.y;
	newPar.goal_x = target.x;
	newPar.goal_y = target.y;
 	
};


void Scenario::printMap() {
	
	int i,j;
	for(j=0; j<scenario_map.height; j++) {
		for(i=0; i<scenario_map.width; i++) {
			cout << scenario_map.coord[i][j].sym;
		}
		cout << endl;
	}

};

vector<Scenario_par>& Scenario::getParam_vect() {

	return scenario_vect;

};

void Scenario::printMap(string& new_file) {
	
	std::ofstream temp;
	temp.open(new_file.c_str());
	if( temp.fail() ) {
		cout << "Print Map to file opening failed.\n";
		exit(1);
	}
	
	temp << "copy from scenario " << name << endl;
	temp << "copy from map " << map_name << endl;
	temp << "height " << scenario_map.height << endl;
	temp << "width " << scenario_map.width << endl;

	int i,j;
	for(j=0; j<scenario_map.height; j++) {
		for(i=0; i<scenario_map.width; i++) {
			temp << scenario_map.coord[i][j].sym;
		}
		temp << endl;
	}
	
};


void Scenario::printMap_point(Pixel& point, string& filename, char symbol) {
	
	std::ofstream temp;
	temp.open(filename.c_str());
	if( temp.fail() ) {
		cout << "Print Map to file opening failed.\n";
		exit(1);
	}

	temp << "Writing path found in scenario " << name << endl;
	temp << "name of the map " << map_name << endl;
	temp << "height " << scenario_map.height << endl;
	temp << "width " << scenario_map.width << endl;

	int i,j;
	for(j=0; j<scenario_map.height; j++) {
		for(i=0; i<scenario_map.width; i++) {
			if (i==point.x && j==point.y) {
				
				temp << symbol;
				scenario_map.coord[i][j].sym = symbol;
				
			}
			else {
				
				temp << scenario_map.coord[i][j].sym;
				
			}
		}
		
		temp << endl;
		
	}

	temp.close();
};

void printMap_point(Scenario& scen, Pixel& point, string& filename, char symbol) {
	
	std::ofstream temp;
	temp.open(filename.c_str());
	if( temp.fail() ) {
		cout << "Print Map to file opening failed.\n";
		exit(1);
	}

	Map map = scen.getMap();

	temp << "Writing path found in scenario " << scen.getScenarioName() << endl;
	temp << "name of the map " << scen.getMapName() << endl;
	temp << "height " << map.height << endl;
	temp << "width " << map.width << endl;

	int i,j;
	for(j=0; j<map.height; j++) {
		for(i=0; i<map.width; i++) {
			if (i==point.x && j==point.y) {
				temp << symbol;				
				map.coord[i][j].sym = symbol;
			}
			else {
				temp << map.coord[i][j].sym;
			}
		}
		
		temp << endl;
		
	}

	temp.close();
};


void printMap_point(Map& map, Pixel& point, string& filename, char symbol) {
	
	std::ofstream temp;
	temp.open(filename.c_str());
	if( temp.fail() ) {
		cout << "Print Map to file opening failed.\n";
		exit(1);
	}

	int i,j;
	for(j=0; j<map.height; j++) {
		for(i=0; i<map.width; i++) {
			if (i==point.x && j==point.y) {
				temp << symbol;				
				map.coord[i][j].sym = symbol;
			}
			else {
				temp << map.coord[i][j].sym;
			}
		}
		
		temp << endl;
		
	}

	temp.close();
};

Scenario_par& Scenario::getScenario_par(int index) {

	return scenario_vect.at(index);

};

Map& Scenario::getMap() {

	return scenario_map;

};

int Scenario::getSize() {

	return scenario_vect.size();

};
	
double Scenario::getOptimal(int index) {


	return scenario_vect.at(index).optimal_length;

};

Pixel Scenario::getSource(int index) {
	
	int x = scenario_vect.at(index).start_x;
	int y = scenario_vect.at(index).start_y;
	Pixel Pix(x, y);
	return Pix;

};

Pixel Scenario::getTarget(int index) {

	int x = scenario_vect.at(index).goal_x;
	int y = scenario_vect.at(index).goal_y;
	Pixel Pix(x, y);
	return Pix;

};



string Scenario::getScenarioName() 
{

	return name;
};

string Scenario::getMapName() {

	return map_name;
};

/*
Scenaro::~Scenario() {

	//scenario_vect.erase(scenario_vect.begin(), scenario_vect.end());

}
*/

// General utilities
void printMap(Map& map, string& filename) {
	
	std::ofstream temp;
	temp.open(filename.c_str());
	if( temp.fail() ) {
		cout << "Print Map to file opening failed.\n";
		exit(1);
	}

	int i,j;
	for(j=0; j<map.height; j++) {
		for(i=0; i<map.width; i++) {
			temp << map.coord[i][j].sym;
		}
		temp << endl;
	}

};

ScenarioList::ScenarioList(std::vector<string>& string_list) {

	for(int i=0; i<string_list.size(); i++) {
		Scenario temp_scenario(string_list[i]);
		scenario_archive.push_back(temp_scenario);
	}

};


ScenarioList::ScenarioList(std::vector<string>& string_list, int tot_scenario_set) {

	for(int i=0; i<tot_scenario_set; i++) {

		if ( i == string_list.size() ) {
			cout << "Maximum scenario structures loaded" << endl;
			break;
		}
		Scenario temp_scenario(string_list[i]);
		scenario_archive.push_back(temp_scenario);
	
	}

};

ScenarioList::ScenarioList(std::vector<string>& string_list, int tot_scenario_set, int scenarios_number) {

	for(int i=0; i<tot_scenario_set; i++) {
		if ( i == string_list.size() ) {
			cout << "Maximum scenario structures loaded" << endl;
			break;
		}
		Scenario temp_scenario(string_list[i], scenarios_number);
		scenario_archive.push_back(temp_scenario);
	}

};

int ScenarioList::getSize() {
	
	return scenario_archive.size();

};


void Scenario::deleteMap() {

	for(int i = 0; i < scenario_map.coord.size(); i++) {
		scenario_map.coord[i].erase( scenario_map.coord[i].begin(), scenario_map.coord[i].end() );
	}

};

double getTime()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);

	double time_in_sec = (tv.tv_sec) + ((double) tv.tv_usec * (double) 10e-7);
	return time_in_sec;

};

/*
ScenarioList::~ScenarioList() {
	
	//scenario_archive.erase(scenario_vect.begin(), scenario_vect.end());
}
*/

} // end namespace PathPlanning
	

	
	
	
	
	
	



	



/* 
	Definition of functions in Scenario.h
*/

#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <sys/time.h>
#include <ctype.h>

#include <boost/filesystem.hpp>

#include <glog/logging.h>

#include "Scenario.h"


namespace GlobalPlanning {


	Map::Map()
	{
		width = length = 0;
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



	Scenario::Scenario(string& filename) {
	
		std::ifstream temp;
		temp.open(filename.c_str());

		if( temp.fail() ) {
			LOG(FATAL) << "Scenario file opening failed";
		}
	
		Scenario_par temp_par;
		
		temp.ignore(INT_MAX, '\n');
		string parameters_line; 
		const int num_skips = 4;

		while( !temp.eof() ) {

		getline(temp, parameters_line);
		stringstream ss;
		ss << parameters_line;
		int skip_param = 0;
		while(skip_param != 3) {
			ss.ignore(INT_MAX, ' ');
			skip_param++;
		}

		ss >> temp_par.start.first;
		ss.ignore(INT_MAX, ' ');
		ss >> temp_par.start.second;
		ss.ignore(INT_MAX, ' ');
		ss >> temp_par.goal.first;
		ss.ignore(INT_MAX, ' ');
		ss >> temp_par.goal.second; 
		ss.ignore(INT_MAX, ' ');
		ss >> temp_par.optimal_length;

		scenario_vect.push_back(temp_par);

		}
		
		temp.close();

		scen_name = filename;
	
		boost::filesystem::path p(filename);
		boost::filesystem::path map_dir(p.parent_path().parent_path().string() + "Maps/"
		
		if( boost::filesystem::exist(map_dir) && boost::filesystem::exist(map_dir) ) 
			map_name = map_dir.string() + p.stem().string();
			LOG(INFO) << "Map name: " << map_name << endl;
		}
		else
			LOG(FATAL) << "Map directory invalid";

		temp.open(map_name.c_str());
		if( temp.fail() ) {
			LOG(FATAL) << "Map file opening failed";
		}
	
		
		temp.ignore(INT_MAX, '\n');
		temp.ignore(INT_MAX, ' ');
		temp >> scenario_map.height;
		ss.ignore(INT_MAX, ' ');
		temp >> scenario_map.width;
		temp.ignore(INT_MAX, '\n');
	
		temp.close();

		if ( !MapToNodes() ) {
			LOG(FATAL) << "Error loading the map";
		}



	};

Scenario::Scenario(string& filename, int scenarios_number) {
	
	std::ifstream temp;
		temp.open(filename.c_str());

		if( temp.fail() ) {
			LOG(FATAL) << "Scenario file opening failed";
		}
	
		Scenario_par temp_par;
		
		temp.ignore(INT_MAX, '\n');
		string parameters_line; 
		const int num_skips = 4;

		int line = 0;
		while( line < scenarios_number ) {

		getline(temp, parameters_line);
		stringstream ss;
		ss << parameters_line;
		int skip_param = 0;
		while(skip_param != 3) {
			ss.ignore(INT_MAX, ' ');
			skip_param++;
		}

		ss >> temp_par.start.first;
		ss.ignore(INT_MAX, ' ');
		ss >> temp_par.start.second;
		ss.ignore(INT_MAX, ' ');
		ss >> temp_par.goal.first;
		ss.ignore(INT_MAX, ' ');
		ss >> temp_par.goal.second; 
		ss.ignore(INT_MAX, ' ');
		ss >> temp_par.optimal_length;

		scenario_vect.push_back(temp_par);
		line++;

		}
		
		temp.close();

		scen_name = filename;
	
		boost::filesystem::path p(filename);
		boost::filesystem::path map_dir(p.parent_path().parent_path().string() + "Maps/"
		
		if( boost::filesystem::exist(map_dir) && boost::filesystem::exist(map_dir) ) 
			map_name = map_dir.string() + p.stem().string();
			LOG(INFO) << "Map name: " << map_name << endl;
		}
		else
			LOG(FATAL) << "Map directory invalid";

		temp.open(map_name.c_str());
		if( temp.fail() ) {
			LOG(FATAL) << "Map file opening failed";
		}
	
		temp.ignore(INT_MAX, '\n');
		temp.ignore(INT_MAX, ' ');
		temp >> scenario_map.height;
		ss.ignore(INT_MAX, ' ');
		temp >> scenario_map.width;
		temp.ignore(INT_MAX, '\n');
	
		temp.close();

		if ( !MapToNodes() ) {
			LOG(FATAL) << "Error loading the map";
		}


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
	

	
	
	
	
	
	



	



/* 
	Definition of functions in Scenario.h
*/

#include <fstream>
#include <sstream>
#include <ctime>

#include <boost/filesystem.hpp>

#include <glog/logging.h>

#include "Scenario.h"

using std::string;
using std::vector;

namespace GlobalPlanning {

	std::ostream& operator <<(std::ostream outs, const pixel point) {
	
		outs << "(" << point.first << "," << point.second << ")";

		return outs;
	
	};

	const pixel& operator +(const pixel& left, const pixel& right) {

		return pixel(left.first + right.first, left.second + right.second);

	}

	const pixel& operator -(const pixel& left, const pixel& right) {

		return pixel(left.first - right.first, left.second - right.second);

	}
	
	std::ostream& operator <<(std::ostream outs, const scenario_param param) {
		
		outs << "Start: " << param.start 
             <<	" Goal: " << param.goal
			 << " Optimal: " << param.optimal;
		
		return outs;

	};

	Map::Map() : width(0), height(0) 
	{
		// deliberately empty
	};

	void Map::TextToMap(const string& map_name) {
	
		std::ifstream temp;
		temp.open(map_name.c_str());
		if( temp.fail() ) {
			LOG(FATAL) << "TextToMap: Map file opening failed.\n";
		}
	
		temp.ignore(INT_MAX, '\n');
		temp.ignore(INT_MAX, ' ');
		temp >> height;
		temp.ignore(INT_MAX, ' ');
		temp >> width;
		temp.ignore(INT_MAX, '\n');
		
		string temp_line;

		coord.reserve(height * width);
	
		for(int j = 0; j < height; j++) {

			getline(temp, temp_line);
			coord.append(temp_line);	

		}
	
		temp.close();

		CHECK_EQ(coord.size(), height * width) << "Map size check failed!";

	};

	bool OutOfBounds(const Map& map, const pixel& point) {

		if ( point.first < 0 || point.first > map.width || 
			 point.second < 0 || point.second > map.height ) {
			return true;
		}
		else {
			return false;
		}

	};

	bool DetectCollision(const Map& map, const pixel& point) {
	
		if ( map.coord[point.second * map.height + point.first] != '.' ) {
			return true;
		}
		else {
			return false;
		}

	};

	void Map::printMap(const string& new_file) const {
	
		std::ofstream temp;
		temp.open(new_file.c_str());
		if( temp.fail() ) {
			LOG(FATAL) << "Print Map to file opening failed.\n";
		}
	
		temp << "map " << map_name << std::endl;
		temp << "height " << height << std::endl;
		temp << "width " << width << std::endl;

		for(int j=0; j < height; j++) {
			for(int i=0; i < width; i++) {
				temp << coord[j * height + i];
			}
			temp << std::endl;
		}

		temp.close();
	
	};


	void Map::printMap_point(const pixel point,
							 const string& filename,
							 const char symbol,
							 const bool mod) {

		CHECK_LT(point.first, width) << "Print Point specified not in map: x invalid";
		CHECK_LT(point.second, height) << "Print Point specified not in map: y invalid";
		CHECK_GE(point.first, 0) << "Print Point specified not in map: x negative";
		CHECK_GE(point.second, 0) << "Print Point specified not in map: y negative";

		std::ofstream temp;
		temp.open(filename.c_str());
		if( temp.fail() ) {
			LOG(FATAL) << "Print Map to file opening failed.\n";
		}

		temp << "map " << map_name << std::endl;
		temp << "height " << height << std::endl;
		temp << "width " << width << std::endl;

		for(int j=0; j < height; j++) {
			for(int i=0; i < width; i++) {
				if (i==point.first && j==point.second) {
					if(mod) {
						coord[point.second * height + point.first] = symbol;
					}
					temp << symbol;
				}
				else {
					temp coord[j * height + i];
				}
			}
		
			temp << std::endl;
		
		}

		temp.close();

	};

	void Map::mod_map_point(Map& map, const pixel& point, const char symbol) {

		CHECK_LT(point.first, width) << "Mod Point specified not in map: x invalid";
		CHECK_LT(point.second, height) << "Mod Point specified not in map: y invalid";
		CHECK_GE(point.first, 0) << "Mod Point specified not in map: x negative";
		CHECK_GE(point.second, 0) << "Mod Point specified not in map: y negative";

		coord[point.second * map.width + point.first] = symbol;

	}

	Scenario::Scenario() {

	};

	Scenario::Scenario(string& filename) {
	
		std::ifstream temp;
		temp.open(filename.c_str());

		if( temp.fail() ) {
			LOG(FATAL) << "Scenario file opening failed";
		}
	
		scenario_param temp_par;
		
		temp.ignore(INT_MAX, '\n');
		string parameters_line; 
		
		// number of parameters to skip before getting to the ones needed
		const int num_skips = 3;

		while( !temp.eof() ) {

			getline(temp, parameters_line);
			stringstream ss;
			ss << parameters_line;
			int skip_param = 0;
			while(skip_param != num_skips) {
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
			ss >> temp_par.optimal;

			param_vect.push_back(temp_par);

		}
		
		temp.close();

		scen_name = filename;
	
		boost::filesystem::path p(filename);
		boost::filesystem::path map_dir(p.parent_path().parent_path().string() + "Maps/"
		
		if( boost::filesystem::exist(map_dir) && boost::filesystem::exist(map_dir) ) 
			map_name = map_dir.string() + p.stem().string();
			LOG(INFO) << "Map name: " << map_name << std::endl;
		}
		else
			LOG(FATAL) << "Map directory invalid";

		scenario_map.TextToMap(map_name);

	};

	Scenario::Scenario(const string& filename, const int scenarios_number) {
	
		std::ifstream temp;
		temp.open(filename.c_str());

		if( temp.fail() ) {
			LOG(FATAL) << "Scenario file opening failed";
		}
	
		scenario_param temp_par;
		
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
			ss >> temp_par.optimal;

			param_vect.push_back(temp_par);
			line++;

		}
		
		temp.close();

		scen_name = filename;
	
		boost::filesystem::path p(filename);
		boost::filesystem::path map_dir(p.parent_path().parent_path().string() + "Maps/"
		
		if( boost::filesystem::exist(map_dir) && boost::filesystem::exist(map_dir) ) 
			map_name = map_dir.string() + p.stem().string();
			LOG(INFO) << "Map name: " << map_name << std::endl;
		}
		else
			LOG(FATAL) << "Map directory invalid";


		scenario_map.TextToMap(map_name);

	};

	
	ScenarioList::ScenarioList(std::vector<string>& string_list) {

		for(int i=0; i<string_list.size(); i++) {

			Scenario temp_scenario(string_list[i]);
			scenario_archive.push_back(temp_scenario);
		}

	};

	ScenarioList::ScenarioList(std::vector<string>& string_list, int scenarios_number) {

		for(int i=0; i < string_list.size(); i++) {
		
			Scenario temp_scenario(string_list[i], scenarios_number);
			scenario_archive.push_back(temp_scenario);
		}

	};

	double getTime()
	{
		struct timeval tv;
		gettimeofday(&tv, NULL);

		double time_in_sec = (tv.tv_sec) + ((double) tv.tv_usec * (double) 10e-7);
		return time_in_sec;

	};



} // end namespace PathPlanning
	

	
	
	
	
	
	



	



/* 
	base class for a global planner
*/


#ifndef SEARCH_AGENT_H
#define SEARCH_AGENT_H


#include "Scenario.hpp"


namespace GlobalPlanning
{

using CoordinateList = std::vector< pixel >;

class SearchAgent
{
	public:
	SearchAgent(char agent_char);
	
	SearchAgent(char agent_char, 
				std::vector<string>& string_list, 
				std::string& direction_mode); 

	SearchAgent(char agent_char,
				std::vector<string>& string_list, 
				std::string& direction_mode, 
				int scenarios_number);

	void setDirections(std::string& allowed);

	virtual const int GetPath(const int map_index,
							  const int par_index,
					    	  CoordinateList& PathFound,
							  float& path_cost,
							  double& elapsed_time,
							  int& nodes_exp ) = 0;

	virtual const int GetPath(Map& map,
							  const pixel& start, 
							  const pixel& goal,
							  CoordinateList& PathFound,
							  float& path_cost,
							  double& elapsed_time,
							  int& nodes_exp ) = 0;

	inline const char getSym() const { return symbol; }
	inline const int getDirectionSize() const { return directions.size() }

	private:
	vector< pixel > directions;
	ScenarioList scenarios;
	char symbol;
	
}


}  // namespace GlobalPlanning

#endif

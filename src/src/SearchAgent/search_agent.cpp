/* 
	Definitions of functions defined in search_agent.h
*/

#include <glog/logging.h>

#include "search_agent.h"


namespace GlobalPlanning
{


	void SearchAgent::setDirections(std::string& allowed) {
	
		if (allowed == "diagonal" ) {
			directions.push_back(pixel(0,1));
			directions.push_back(pixel(1,0));
			directions.push_back(pixel(0,-1));
			directions.push_back(pixel(-1,0));
			directions.push_back(pixel(-1,-1));
			directions.push_back(pixel(1,1));
			directions.push_back(pixel(-1,1));
			directions.push_back(pixel(1,-1));
		}
		else if( allowed == "cardinal" ) {
			directions.push_back(pixel(0,1));
			directions.push_back(pixel(1,0));
			directions.push_back(pixel(0,-1));
			directions.push_back(pixel(-1,0));
		}
		else {
			LOG(ERROR) << "Wrong direction mode typed, available: 'diagonal' or 'cartesian' ";
		}

	};

	SearchAgent::SearchAgent() {
		// deliberately empty
	};

	SearchAgent::SearchAgent(char agent_char, 
							 std::vector<string>& string_list, 
							 std::string& allowed) : symbol(agent_char)  {

		setDirections(allowed);
		scenarios = ScenarioList(string_list);
	
	};

	SearchAgent::SearchAgent(char agent_char, 
							 std::vector<string>& string_list, 
							 std::string& allowed, 
							 int scenarios_number) : symbol(agent_char) {

		setDirections(allowed);
		scenarios = ScenarioList(string_list, scenarios_number);
		
	};

	float Astar::CalculateCost(CoordinateList& Path) {

		float cost = 0;
		pixel temp();
	
		for(int i=0; i<Path.size()-1; i++) {
			temp = Path[i+1] - Path[i];
			for(int j=0; j < direction.size(); j++) {
				   if(temp == direction[j]) {
				   dir = j;
				   break;
				}
			}
			if ( dir < 4 ) {
				cost += std::get<0>(scale_param);
			}
			else if( dir >= 4 ) {
				cost += std::get<1>(scale_param);
			}
			
		}

		return cost;

	};
	

}// end of Supervisor namespace implementation






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
							 std::string& allowed) {

		symbol = agent_char;
		setDirections(allowed);
		scenarios = ScenarioList(string_list);
	
	};

	SearchAgent::SearchAgent(char agent_char, 
							 std::vector<string>& string_list, 
							 std::string& allowed, 
							 int scenarios_number) {

		symbol = agent_char;
		setDirections(allowed);
		scenarios = ScenarioList(string_list, scenarios_number);
		
	};

	

}// end of Supervisor namespace implementation






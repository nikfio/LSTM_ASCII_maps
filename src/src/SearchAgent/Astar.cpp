/*
	Definition of agent defined in Astar.h
*/

#include <glog/logging.h>

#include <Astar.h>


DECLARE_int32(timeout);

namespace GlobalPlanning {

	AstarNode::AstarNode()
			    : point(pixel(0,0)), parent(nullptr), G(0), H(0)
	{
		// deliberately empty
	};


	AstarNode::AstarNode(pixel new_point) 
				: point(new_point), parent(nullptr), G(0), H(0)
	{
		// deliberately empty
	};

	AstarNode::AstarNode(pixel new_point, AstarNode* parent_)
				: point(new_point), parent(parent_), G(0), H(0)
	{
		// deliberately empty
	};

	double AstarNode::getScore()
	{
		return G + H;
	};

	AstarNode AstarNode::operator = (const AstarNode& equal)
	{
		point = equal.point;
		G = equal.G;
		H = equal.H;
		parent = equal.parent;
	};

	const bool AstarNode::operator == (const AstarNode& curr)
	{
		if( pix==curr.pix && G==curr.G && H==curr.H && parent==curr.parent)
			return true;
		else
			return false;
	};

	const pixel& getDelta(const pixel& current, const pixel& goal)
	{
		return pixel( abs(source.x - target.x),  abs(source.y - target.y) );
	};


	const float manhattan(const pixel& current, const pixel& target, const cost_scale& scale_param)
	{
		pixel delta = getDelta(source, target));
		return static_cast<float>(std::get<2>(scale_param) * (std::get<0>(scale_param) * (delta.first + delta.second)));
	};

	const float octile(const pixel& current, const pixel& target, const cost_scale& scale_param) 
	{
		pixel delta = getDelta(source, target));
		float cartesian_moves = std::get<0>(scale_param) * (delta.first + delta.second);
		float save_diagonal_moves = (std::get<1>(scale_param) - 2*std::get<0>(scale_param)) * std::min(delta.first, delta.second);
		return static_cast<float>(std::get<2>(scale_param) * ( cartesian_moves + save_diagonal_moves );
	}

	const float euclidean(const pixel& current, const pixel& goal, const cost_scale& scale_param)
	{
		pixel delta = getDelta(source, target));
		return static_cast<float>( std::get<2>(scale_param) * sqrt(pow(delta.first, 2) + pow(delta.first, 2)) );
	};

	void Astar::setHeuristic(std::string& method) {

		if( method == "manhattan" ) {
			heuristic_func = std::bind(&manhattan, _1, _2, _3);
		}
		else if ( method == "diagonal" ) {
			heuristic_func = std::bind(&diagonal, _1, _2, _3);
		}
		else if ( method == "euclidean" ) {
			heuristic_func = std::bind(&manhattan, _1, _2, _3);
		}
		else {
			LOG(FATAL) << "Not valid heuristic selected: 'manhattan', 'diagonal' and 'euclidean' available";
		}

	}


	Astar::Astar(char agent_char, 
		  		 std::vector<string>& string_list, 
		  		 std::string& direction_mode,
		  		 const cost_scale scale_param,
		  		 std::string& heuristic_func) 
				: SearchAgent(agent_char, string_list, direction_mode), 
				  scale_param(scale_param)
	{
		setHeuristic(heuristic_func);
	};

	Astar::Astar(char agent_char, 
		  		 std::vector<string>& string_list, 
		  		 std::string& direction_mode,
		  		 int scenarios_number,
		  		 cost_scale scale_param,
		  		 std::string& heuristic_func) 
				: SearchAgent(agent_char, string_list, direction_mode, scenarios_number), 
					scale_param(scale_param)
	{
		setHeuristic(heuristic_func);
	};


	int  Astar::find_index(AstarNodeSet& set, AstarNode* curr) {

	for (int i = 0; i < set.size(); i++) {
		if (set[i] == curr) {
			break;
		}
	}

	return i;	
	
	};

	AstarNode* Astar::findNodeInSet(AstarNodeSet& set, pixel& point_check)
	{
		for (int i =0; i<AstarNodes.size(); i++) {
		    if (AstarNodes[i]->point == point_check) {
		        return AstarNodes[i];
		    }
		}
		return nullptr;
	};

	const int GetPath(const int map_index,
					  const int par_index,
					  CoordinateList& PathFound,
					  float& path_cost,
					  double& elapsed_time,
				      int& nodes_exp ) const

  	{
	
		CHECK_LT(map_index, scenarios.getSize()) << "Map request invalid: out of size";

		Scenario scenario = scenarios.getScenario(map_index);

		CHECK_LT(par_index, scenario.getSize()) << "Scenario parameters request invalid: out of size";

		Map map = scenario.getMap();
	
		pixel start = scenario.getStart(par_index);
		pixel goal = scenario.getGoal(par_index);
	
		VLOG(1) << "Agent Astar called for Map: " << scenario.getMapName() << std::endl
		 		<< "Scenario parameters index: " << par_index << std::endl;
				<< "Goal :" << target << std::endl;
		 		<< "Source :" << source << std::endl;
				<< "Optimal length supposed " << scenario.getOptimal(par_index);

		if(!empty(OpenSet))
			OpenSet.clear();
	
		if(!empty(ClosedSet))
			ClosedSet.clear();
		
		AstarNode *current, *successor;
		AstarNode *start_node = new AstarNode(start);
		OpenSet.push_back(SourceAstarNode);

		path_cost = nodes_exp = 0;
		
		float totalCost = 0;
		double start = getTime();
		while (!openSet.empty()) {
		
			current = openSet.front();
			for(int i=0; i < OpenSet.size(); i++)   {
				if (openSet[i]->getAstarScore() < current->getScore()) {
				    current = OpenSet[i];
				}
			}
		
			if (current->point == goal) {
				LOG(INFO) << "GOAL FOUND";
				break;
			}
		
			ClosedSet.push_back(current); 
			if( find_index(openSet, current) < openSet.size() ) {
				VLOG(2) << "erasing AstarNode in OpenSet: "
						<< current->point;
	
		    	OpenSet.erase(openSet.begin() + find_index(openSet, current));
			}
			else {
				VLOG(2) << "Current AstarNode is not on OPEN list: "
						<< current->point;
			}

			if( nodes_exp++ > timeout )
				break;
		
			for (int i = 0; i < direction.size(); i++) {
		        pixel touch_point(current->point + directions[i]);
			
				if ( OutOfBounds(map, touch_point) ) {
					VLOG(2) << "Agent exceeded map extreme bounds";
					continue;
				}			

		       	if ( DetectCollision(map, touch_point) ) {
					VLOG(2) << "Collision DETECTED\n";
					continue;
		        }
			
				if ( findNodeInSet(ClosedSet, touch_point) ) {
					continue;
				}
			
				if ( directions.size() == 8 ) {
					if( i < 4 ) {
						totalCost = current->G + std::get<0>(scale_param);
					}
					else if ( i >= 4 ) 
					{
						totalCost = current->G + std::get<1>(scale_param);
					}
				}
				else if ( directions.size() == 4 ) {
					totalCost = current->G + std::get<0>(scale_param);
				}

				successor = findNodeInSet(OpenSet, touch_point);
				if (successor == nullptr ) {
					successor = new AstarNode(touch_point, current);		
					successor->G = totalCost;
					successor->H = heuristic(successor->point, goal);
					OpenSet.push_back(successor);		
				}
				else if (totalCost < successor->G) {
					successor->parent = current;
					successor->G = totalCost;
					successor->H = heuristic(successor->point, goal);
		        }
			
			}	

		
		}
		double end = getTime();	
		elapsed_time = end - start;	

		if(!empty(PathFound))
			PathFound.clear();
		
		while (current != nullptr) {
		   	PathFound.push_back(current->pix);
		   	current = current->parent;
		}
	
		reverse(PathFound.begin(), PathFound.end());
	
		path_cost = CalculateCost(PathFound);
			
		delete start_node;
		OpenSet.clear();
		ClosedSet.clar();

		if( VLOG_IS_ON(2) ) {_
			string test_file = "test";
			PrintPath(Path, scenario, test_file);
		}
		
		
		if(PathFound.back() != goal ) 
			return 0;
		else {
			VLOG(1) << "Agent A star cost: " << path_cost 
			        << "	Optimal: " << scenario.getOptimal(par_index) << endl;
			return 1;
		}

	};


	const int GetPath(Map& map,
					  const pixel& start, 
					  const pixel& goal,
					  CoordinateList& PathFound,
					  float& path_cost,
					  double& elapsed_time,
					  int& nodes_exp ) 
	{
		
	
		VLOG(1) << "Agent Astar called for Map: " << scenario.getMapName() << std::endl
		 		<< "Scenario parameters index: " << par_index << std::endl;
				<< "Goal :" << target << std::endl;
		 		<< "Source :" << source << std::endl;
				<< "Optimal length supposed " << scenario.getOptimal(par_index);

		if(!empty(OpenSet))
			OpenSet.clear();
	
		if(!empty(ClosedSet))
			ClosedSet.clear();
		
		AstarNode *current, *successor;
		AstarNode *start_node = new AstarNode(start);
		OpenSet.push_back(start_node);

		path_cost = nodes_exp = 0;
		
		float totalCost = 0;
		double start = getTime();
		while (!openSet.empty()) {
		
			current = openSet.front();
			for(int i=0; i < OpenSet.size(); i++)   {
				if (openSet[i]->getAstarScore() < current->getScore()) {
				    current = OpenSet[i];
				}
			}
		
			if (current->point == goal) {
				LOG(INFO) << "GOAL FOUND";
				break;
			}
		
			ClosedSet.push_back(current); 
			if( find_index(openSet, current) < openSet.size() ) {
				VLOG(2) << "erasing AstarNode in OpenSet: "
						<< current->point;
	
		    	OpenSet.erase(openSet.begin() + find_index(openSet, current));
			}
			else {
				VLOG(2) << "Current AstarNode is not on OPEN list: "
						<< current->point;
			}

			if( nodes_exp++ > timeout )
				break;
		
			for (int i = 0; i < direction.size(); i++) {
		        pixel touch_point(current->point + directions[i]);
			
				if ( OutOfBounds(map, touch_point) ) {
					VLOG(2) << "Agent on map extreme bounds";
					continue;
				}			

		       	if ( DetectCollision(map, touch_point) ) {
					VLOG(2) << "Collision DETECTED\n";
					continue;
		        }
			
				if ( findNodeInSet(ClosedSet, touch_point) ) {
					continue;
				}
			
				if ( directions.size() == 8 ) {
					if( i < 4 ) {
						totalCost = current->G + std::get<0>(scale_param);
					}
					else if ( i >= 4 ) 
					{
						totalCost = current->G + std::get<1>(scale_param);
					}
				}
				else if ( directions.size() == 4 ) {
					totalCost = current->G + std::get<0>(scale_param);
				}

				successor = findNodeInSet(OpenSet, touch_point);
				if (successor == nullptr ) {
					successor = new AstarNode(touch_point, current);		
					successor->G = totalCost;
					successor->H = heuristic(successor->point, goal);
					OpenSet.push_back(successor);		
				}
				else if (totalCost < successor->G) {
					successor->parent = current;
					successor->G = totalCost;
					successor->H = heuristic(successor->point, goal);
		        }
			
			}	

		
		}
		double end = getTime();	
		elapsed_time = end - start;	

		if(!empty(PathFound))
			PathFound.clear();
		
		while (current != nullptr) {
		   	PathFound.push_back(current->pix);
		   	current = current->parent;
		}
	
		reverse(PathFound.begin(), PathFound.end());

		path_cost = CalculateCost(PathFound);
		
		delete start_node;
		OpenSet.clear();
		ClosedSet.clar();

		if( VLOG_IS_ON(2) ) {_
			string test_file = "test";
			PrintPath(Path, scenario, test_file);
		}
		
		if(PathFound.back() != goal ) 
			return 0;
		else {
			VLOG(1) << "Agent A star cost: " << path_cost 
			        << "	Optimal: " << scenario.getOptimal(par_index) << endl;
			return 1;
		}

	};

	
void PrintPath(CoordinateList& PathFound, Scenario& scenario, string& filename) {


	ofstream temp;
	temp.open(filename.c_str());
	if( temp.fail() ) {
		cout << "Print Map to file opening failed.\n";
		exit(1);
	}

	Map scenario_map = scenario.getMap();
	
	temp << "Writing path found in scenario " << scenario.getScenarioName() << endl;
	temp << "name of the map " << scenario.getMapName() << endl;
	temp << "height " << scenario_map.height << endl;
	temp << "width " << scenario_map.width << endl;
	
	int i,j;
	for(j=0; j<scenario_map.height; j++) {
		int index = find_AstarNode_coord(i, j, PathFound); 
		if ( index != PathFound.size() ) {
			if( index == 0 )
				temp << 'S';
			else if (index == PathFound.size() - 1)
				temp << 'G';
			else
				temp << 'A';
		}
		temp << endl;
	}
	
	temp.close();
	
};

void PrintPath(CoordinateList& PathFound, Scenario& scenario, string& filename, char symbol) {


	ofstream temp;
	temp.open(filename.c_str());
	if( temp.fail() ) {
		cout << "Print Map to file opening failed.\n";
		exit(1);
	}


	Map map = scenario.getMap();
	
	temp << "Writing path found in scenario " << scenario.getScenarioName() << endl;
	temp << "name of the map " << scenario.getMapName() << endl;
	temp << "height " << map.height << endl;
	temp << "width " << map.width << endl;
	

	int i,j;
	for(j=0; j<map.height; j++) {
		for(i=0; i<map.width; i++) {
			int index = find_AstarNode_coord(i, j, PathFound); 
			if ( index != PathFound.size() ) {
				if( index == 0 )
					temp << 'S';
				else if (index == PathFound.size() - 1)
					temp << 'G';
				else
					temp << symbol;
			}
		}
		temp << endl;
	}
	
	temp.close();
	


};


void PrintPath(CoordinateList& PathFound, Map& map, string& filename, char symbol) {


	ofstream temp;
	temp.open(filename.c_str());
	if( temp.fail() ) {
		cout << "Print Map to file opening failed.\n";
		exit(1);
	}
	
	int i,j;
	for(j=0; j<map.height; j++) {
		for(i=0; i<map.width; i++) {
			int index = find_AstarNode_coord(i, j, PathFound); 
			if ( index != PathFound.size() ) {
				if( index == 0 )
					temp << 'S';
				else if (index == PathFound.size() - 1)
					temp << 'G';
				else
					temp << symbol;
			}
			else
			{
				temp << map.coord[i][j].sym;
			}
		}
		temp << endl;
	}
	
	temp.close();
	


};

	
}; // namespace GlobalPlanning

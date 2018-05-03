/*
	Definition of agent defined in Astar.h
*/

#include <glog/logging.h>

#include <Astar.h>


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

	friend AstarNode AstarNode::operator = (const AstarNode& equal)
	{
		point = equal.point;
		G = equal.G;
		H = equal.H;
		parent = equal.parent;
	};

	friend const bool AstarNode::operator == (const AstarNode& curr)
	{
		if( pix==curr.pix && G==curr.G && H==curr.H && parent==curr.parent)
			return true;
		else
			return false;

	}

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
		  		 std::tuple<float,float,float> cost_scale,
		  		 std::string& heuristic_func) 
				: SearchAgent(agent_char, string_list, direction_mode), 
					cost_scale(cost_scale)
	{
		setHeuristic(heuristic_func);
	};

	Astar::Astar(char agent_char, 
		  		 std::vector<string>& string_list, 
		  		 std::string& direction_mode,
		  		 int scenarios_number,
		  		 std::tuple<float,float,float> cost_scale,
		  		 std::string& heuristic_func) 
				: SearchAgent(agent_char, string_list, direction_mode, scenarios_number), 
					cost_scale(cost_scale)
	{
		setHeuristic(heuristic_func);
	};


	int  Astar::find_index(AstarNodeSet& set, AstarNode* curr) {

	for (int i = 0; i < set.size(); i++) {
		    if (set[i] == curr) {
		        return i;
		    }
		}
		if(i==AstarNodes.size()) {
		return i;
		}

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
		
		double start = getTime();
		while (!openSet.empty()) {
		
			current = openSet.front(); int i;
			for(i=0; i<openSet.size(); i++)   {
				if (openSet[i]->getAstarScore() < current->getAstarScore()) {
				    current = openSet[i];
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

			nodes_exp++;

			for (int i = 0; i < direction.size(); i++) {
		        pixel touch_point(current->pix + direction[i]);
			
				if ( OutOfBounds(map, touch_point) ) {
					VLOG(1) << "Agent on map extreme bounds";
					continue;
				}

				AstarNode *newAstarNode = &PathMap.coord[newCoordinates.x][newCoordinates.y];
			

		       	if ( detectCollision(newAstarNode) ) {
					//cout << "Collision DETECTED\n";
					continue;
		        	}
			
			
			
				if ( findAstarNodeOnList(closedSet, newCoordinates) ) {
					continue;
				}
			
				if ( direction.size() == 8 ) {
					if( i < 4 ) {
						totalCost = current->G + std::get<0>(cost_scale);
					}
					else if ( i >= 4 ) 
					{
						totalCost = current->G + std::get<1>(cost_scale);
					}
				}
				else if ( direction.size() == 4 ) {
					totalCost = current->G + std::get<0>(cost_scale);
				}

				successor = findAstarNodeOnList(openSet, newCoordinates);
				if (successor == nullptr ) {
					successor = new AstarNode(newCoordinates, current);		
					successor->G = totalCost;
					successor->H = ComputeHeuristic(successor->pix, target);
					openSet.push_back(successor);	
				
				}
					else if (totalCost < successor->G) {
					successor->parent = current;
					successor->G = totalCost;
					successor->H = ComputeHeuristic(successor->pix, target);
		        	}
			
			}	

		
		}
		double end = getTime();	

		if(!empty(PathFound))
			PathFound.clear();
		
		while (current != nullptr) {
		   	PathFound.push_back(current->pix);
		   	current = current->parent;
		}

		elapsed_time = end - start;	
	
		reverse(PathFound.begin(), PathFound.end());
	
		if(failed || Path.back() != target ) 
			return 0;

		PathCost = CalculateCost(Path);

		if( VLOG_IS_ON(2) ) {
			string test_file = "last_test";
			PrintPath(Path, scenario, test_file);
		}

		delete start_node;
		
		LOG(INFO) << "Comparison:    " <<    "Agent A star cost: " << totalCost 
			 << "		Calculation method cost: " << PathCost << endl;
	
		if (totalCost < MAX_LENGTH && PathCost < MAX_LENGTH ) {		
			return 1;
		}
		else {			
			return 0;
		}
	
	};


int Astar::search_path(float& PathCost, CoordinateList& Path, pixel& source_, pixel& target_, int& AstarNodes_exp) {
	
	if( scenario.getSize() == 0 ) {
		cerr << "No scenarios loaded\n";
	}
	
	Map PathMap = scenario.getMap();
	
	#ifdef DEBUG_LEV_1
        cout << "Agent Astar called for scenario" << scenario.getScenarioName() 
	     << endl << "Map " << scenario.getMapName() << endl;
	cout << "Goal :" << target_ << endl;
	cout << "Source :" << source_ << endl;
	#endif

	AstarNode *current;
    	AstarNodeSet openSet, closedSet;
	AstarNode *SourceAstarNode = new AstarNode(source_);
    	openSet.push_back(SourceAstarNode);
	AstarNode *successor; 

	float totalCost = 0;

	AstarNodes_exp = 0;
	while (!openSet.empty()) {
		
		current = openSet.front(); int i;
		for(i=0; i<openSet.size(); i++)   {
		    if (openSet[i]->getAstarScore() < current->getAstarScore()) {
		        current = openSet[i];
		    }
		}
		
		if (current->pix == target_ ) {
		    #ifdef DEBUG_LEV_1
		    cout << "GOAL AstarNode FOUND  "  << endl;
		    #endif
		    break;
		}
		
		closedSet.push_back(current); 
		if( find_index(openSet, current) < openSet.size() ) {
			//cout << "erasing AstarNode in openSet: "<< current.pix.x << ","<< current.pix.y <<endl;		
        		openSet.erase(openSet.begin() + find_index(openSet, current));
		}
		else {
			//cout << "Current AstarNode is not on OPEN list \n";
		}

		AstarNodes_exp++;

		for (int i = 0; i < direction.size(); i++) {
                	pixel newCoordinates(current->pix + direction[i]);
			
			if ( OutOfBounds(PathMap, newCoordinates) ) {
				//cout << "Agent on map extreme bounds" << endl;
				continue;
			}

			AstarNode *newAstarNode = &PathMap.coord[newCoordinates.x][newCoordinates.y];
			

           	if ( detectCollision(newAstarNode) ) {
				//cout << "Collision DETECTED\n";
				continue;
            	}
			
			
			if ( findAstarNodeOnList(closedSet, newCoordinates) ) {
				continue;
			}
			
			if ( direction.size() == 8 ) {
				if( i < 4 ) {
					totalCost = current->G + 1;
				}
				else if ( i >= 4 ) 
				{
					totalCost = current->G + sqrt(2);
				}
			}
			else if ( direction.size() == 4 ) {
				totalCost = current->G + 1;
			}

			successor = findAstarNodeOnList(openSet, newCoordinates);
		    	if (successor == nullptr ) {
				successor = new AstarNode(newCoordinates, current);		
				successor->G = totalCost;
				successor->H = ComputeHeuristic(successor->pix, target_);
				openSet.push_back(successor);		
		    	}
		    	else if (totalCost < successor->G) {
				successor->parent = current;
				successor->G = totalCost;
				successor->H = ComputeHeuristic(successor->pix, target_);
            		}
			
		}	
		
		
		
	}

	while (current != nullptr) {
        	Path.push_back(current->pix);
        	current = current->parent;
    	}

	reverse(Path.begin(), Path.end());
	string test_new = "test_new_func";
	PrintPath(Path, scenario, test_new);

	delete SourceAstarNode;

	releaseAstarNodes(openSet);
	releaseAstarNodes(closedSet);

	PathCost = CalculateCost(Path);

	#ifdef DEBUG_LEV_1
	
	cout << "Comparison:    " <<    "Agent A star cost: " << totalCost 
	     << "		Calculation method cost: " << PathCost << endl;
	
	#endif

	if (totalCost < MAX_LENGTH && PathCost < MAX_LENGTH) {		
		return 1;
	}
	else {			
		return 0;
	}
	
};


int Astar::search_extra_path(float& PathCost, CoordinateList& Path, pixel& source_, pixel& target_, int& AstarNodes_exp) {

	scenario_param extra;
	BuildPar(extra, source_, target_);
	int index = scenario.getSize();
	scenario.scenario_vect.push_back(extra);
	double elapsed_time;	

	if ( search_path(PathCost, Path, index, elapsed_time, AstarNodes_exp) == 1 )
		return 1;
	else
		return 0;

};



bool OutOfBounds(Map& map, pixel& next) {

	if ( next.x < 0 || next.x > map.width || 
	     next.y < 0 || next.y > map.height ) {
		return true;
	}
	else {
		return false;
	}

};

bool detectCollision(AstarNode& newAstarNode) {
	
	if ( newAstarNode.sym != '.' ) {
		return true;
	}
	else {
		return false;
	}

};

bool detectCollision(AstarNode* newAstarNode) {
	
	if ( newAstarNode->sym != '.' ) {
		return true;
	}
	else {
		return false;
	}

};


void Astar::releaseAstarNodes(AstarNodeSet& AstarNodes)
{

    
    for(int i=0; i<AstarNodes.size(); i++) {
        AstarNodes.pop_back();
    }
    
   
};

AstarNode* Astar::findAstarNodeOnList(AstarNodeSet& AstarNodes, pixel& coordinates)
{
    for (int i =0; i<AstarNodes.size(); i++) {
        if (AstarNodes[i]->pix == coordinates) {
            return AstarNodes[i];
        }
    }
    return nullptr;
};


pixel getDelta(pixel source, pixel target)
{
    pixel delta( abs(source.x - target.x),  abs(source.y - target.y) );
    return delta;
};

float euclidean(pixel source, pixel target)
{
    pixel delta = std::move(getDelta(source, target));
    return static_cast<float>( 25 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)) );
};

float Astar::ComputeHeuristic(pixel source, pixel target)
{
    if( HeuristicMethod == "euclidean"  )  {
    	return euclidean(source, target);
    }
    else {
	cout << "Heuristic Method selected is not available\n";
        return 0;
    }

};


int find_AstarNode_coord(int map_x, int map_y, CoordinateList& Path) 
{

int i;
for(i = 0; i<Path.size(); i++) {
	if ( Path[i].x == map_x && Path[i].y == map_y ){
		break;
	}
}

return i;


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

float CalculateCost(CoordinateList& Path, string& direction_mode) {

	int i, j, dir, size;
	float cost = 0;
	pixel temp;

	if( direction_mode == "diagonal" ) {
		size = 8;
	}
	else if ( direction_mode == "cardinal" ) {
		size = 4;
	}

	vector<pixel> direction;
	setDirections(direction, direction_mode);

	for(i=0; i<Path.size()-1; i++) {
		//temp = pixel(Path[i+1].x - Path[i].x, Path[i+1].y - Path[i].y);
		temp = Path[i+1] - Path[i];
		for( j=0; j<size; j++) {
		       if(temp == direction[j]) {
			   dir = j;
			   break;
			}
		}
		if ( dir < 4 ) {
			cost += 1;
		}
		else if( dir >= 4 ) {
			cost += sqrt(2);
		}

	}

	return cost;

};

float Astar::CalculateCost(CoordinateList& Path) {

	int i, j, dir;
	float cost = 0;
	pixel temp;
	
	for(i=0; i<Path.size()-1; i++) {
		//temp = pixel(Path[i+1].x - Path[i].x, Path[i+1].y - Path[i].y);
		temp = Path[i+1] - Path[i];
		for( j=0; j < direction.size(); j++) {
		       if(temp == direction[j]) {
			   dir = j;
			   break;
			}
		}
		if ( dir < 4 ) {
			cost += 1;
		}
		else if( dir >= 4 ) {
			cost += sqrt(2);
		}
			
	}

	return cost;

};

int Astar::getDirectionSize() {

	return direction.size();

};

bool findpixel(int x, int y, LocalMap& occ) {

	for(int i = 0; i < occ.size(); i++) {
		if( occ[i].pix.x == x && occ[i].pix.y == y ) {
			return true;
		}
	
	}	

	return false;

};


	
}; // namespace GlobalPlanning

/*
	Definition of agent defined in Astar.h
*/

#include <glog/logging.h>

#include <Astar.h>



namespace GlobalPlanning {

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


Astar::Astar(string& direction_mode)
{
	setHeuristic("euclidean");
	setDirections(direction, direction_mode);
};

Astar::Astar(Scenario& init_scen, string& direction_mode) {

	setHeuristic("euclidean");
	setDirections(direction, direction_mode);	
	scenario = init_scen;

};

void Astar::setHeuristic(string new_heuristic)
{
    HeuristicMethod = new_heuristic;
}


int  Astar::find_index(NodeSet& nodes, Node* curr) {

int i;
for (i = 0; i<nodes.size(); i++) {
        if (nodes[i] == curr) {
            return i;
        }
    }
    if(i==nodes.size()) {
    return i;
    }
}

int Astar::search_path(float& PathCost, CoordinateList& Path, int& index, double& elapsed_time, int& nodes_exp) {
	
	if( scenario.getSize() == 0 ) {
		cerr << "No scenarios loaded\n";
	}

	Scenario_par temp_param = scenario.getScenario_par(index);

	Map PathMap = scenario.getMap();
	
	pixel source, target;
	source.x = temp_param.start_x;
	source.y = temp_param.start_y;
	target.x = temp_param.goal_x;
	target.y = temp_param.goal_y;
	
	#ifdef DEBUG_LEV_1
        cout << "Agent Astar called for scenario" << scenario.getScenarioName() 
	     << endl << "Map " << scenario.getMapName() << endl;
	cout << "Scenario's specifics: "<< endl << "Index " << index << endl;
	cout << "Goal :" << target << endl;
	cout << "Source :" << source << endl;
	cout << "Optimal length supposed " << temp_param.optimal_length << endl;
	#endif
		
	Node *current;
    	NodeSet openSet, closedSet;
	Node *SourceNode = new Node(source);
    	openSet.push_back(SourceNode);
	Node *successor; 

	float totalCost = 0;
	bool failed = false;

	double start = getTime();

	nodes_exp = 0;
	while (!openSet.empty()) {
		
		current = openSet.front(); int i;
		for(i=0; i<openSet.size(); i++)   {
		    if (openSet[i]->getAstarScore() < current->getAstarScore()) {
		        current = openSet[i];
		    }
		}
		
		if (current->pix == target ) {
		    #ifdef DEBUG_LEV_1
		    cout << "GOAL NODE FOUND  "  << endl;
		    #endif
		    break;
		}
		
		closedSet.push_back(current); 
		if( find_index(openSet, current) < openSet.size() ) {
			//cout << "erasing node in openSet: "<< current.pix.x << ","<< current.pix.y <<endl;		
        		openSet.erase(openSet.begin() + find_index(openSet, current));
		}
		else {
			//cout << "Current node is not on OPEN list \n";
		}

		nodes_exp++;

		for (int i = 0; i < direction.size(); i++) {
                	pixel newCoordinates(current->pix + direction[i]);
			
			if ( OutOfBounds(PathMap, newCoordinates) ) {
				//cout << "Agent on map extreme bounds" << endl;
				continue;
			}

			Node *newNode = &PathMap.coord[newCoordinates.x][newCoordinates.y];
			

           	if ( detectCollision(newNode) ) {
				//cout << "Collision DETECTED\n";
				continue;
            	}
			
			
			
			if ( findNodeOnList(closedSet, newCoordinates) ) {
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

			successor = findNodeOnList(openSet, newCoordinates);
		    	if (successor == nullptr ) {
				successor = new Node(newCoordinates, current);		
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
	
	while (current != nullptr) {
        	Path.push_back(current->pix);
        	current = current->parent;
    	}

	double end = getTime();

	elapsed_time = end - start;	
	
	reverse(Path.begin(), Path.end());
	
	if(failed || Path.back() != target ) 
		return 0;

	PathCost = CalculateCost(Path);

	#ifdef DEBUG_LEV_1
	string test_file = "last_test";
	PrintPath(Path, scenario, test_file);
	delete SourceNode;
	releaseNodes(openSet);
	releaseNodes(closedSet);
	
	cout << "Comparison:    " <<    "Agent A star cost: " << totalCost 
	     << "		Calculation method cost: " << PathCost << endl;
	
	#endif

	
	if (totalCost < MAX_LENGTH && PathCost < MAX_LENGTH ) {		
		return 1;
	}
	else {			
		return 0;
	}
	
};


int Astar::search_path(float& PathCost, CoordinateList& Path, pixel& source_, pixel& target_, int& nodes_exp) {
	
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

	Node *current;
    	NodeSet openSet, closedSet;
	Node *SourceNode = new Node(source_);
    	openSet.push_back(SourceNode);
	Node *successor; 

	float totalCost = 0;

	nodes_exp = 0;
	while (!openSet.empty()) {
		
		current = openSet.front(); int i;
		for(i=0; i<openSet.size(); i++)   {
		    if (openSet[i]->getAstarScore() < current->getAstarScore()) {
		        current = openSet[i];
		    }
		}
		
		if (current->pix == target_ ) {
		    #ifdef DEBUG_LEV_1
		    cout << "GOAL NODE FOUND  "  << endl;
		    #endif
		    break;
		}
		
		closedSet.push_back(current); 
		if( find_index(openSet, current) < openSet.size() ) {
			//cout << "erasing node in openSet: "<< current.pix.x << ","<< current.pix.y <<endl;		
        		openSet.erase(openSet.begin() + find_index(openSet, current));
		}
		else {
			//cout << "Current node is not on OPEN list \n";
		}

		nodes_exp++;

		for (int i = 0; i < direction.size(); i++) {
                	pixel newCoordinates(current->pix + direction[i]);
			
			if ( OutOfBounds(PathMap, newCoordinates) ) {
				//cout << "Agent on map extreme bounds" << endl;
				continue;
			}

			Node *newNode = &PathMap.coord[newCoordinates.x][newCoordinates.y];
			

           	if ( detectCollision(newNode) ) {
				//cout << "Collision DETECTED\n";
				continue;
            	}
			
			
			if ( findNodeOnList(closedSet, newCoordinates) ) {
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

			successor = findNodeOnList(openSet, newCoordinates);
		    	if (successor == nullptr ) {
				successor = new Node(newCoordinates, current);		
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

	delete SourceNode;

	releaseNodes(openSet);
	releaseNodes(closedSet);

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


int Astar::search_extra_path(float& PathCost, CoordinateList& Path, pixel& source_, pixel& target_, int& nodes_exp) {

	Scenario_par extra;
	BuildPar(extra, source_, target_);
	int index = scenario.getSize();
	scenario.scenario_vect.push_back(extra);
	double elapsed_time;	

	if ( search_path(PathCost, Path, index, elapsed_time, nodes_exp) == 1 )
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

bool detectCollision(Node& newNode) {
	
	if ( newNode.sym != '.' ) {
		return true;
	}
	else {
		return false;
	}

};

bool detectCollision(Node* newNode) {
	
	if ( newNode->sym != '.' ) {
		return true;
	}
	else {
		return false;
	}

};


void Astar::releaseNodes(NodeSet& nodes)
{

    
    for(int i=0; i<nodes.size(); i++) {
        nodes.pop_back();
    }
    
   
};

Node* Astar::findNodeOnList(NodeSet& nodes, pixel& coordinates)
{
    for (int i =0; i<nodes.size(); i++) {
        if (nodes[i]->pix == coordinates) {
            return nodes[i];
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


int find_node_coord(int map_x, int map_y, CoordinateList& Path) 
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
		int index = find_node_coord(i, j, PathFound); 
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
			int index = find_node_coord(i, j, PathFound); 
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
			int index = find_node_coord(i, j, PathFound); 
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

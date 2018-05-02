
#include <functional>

#include <glog/logging.h>

#include "search_agent.h"



namespace GlobalPlanning {

class AstarNode {

	public:
	AstarNode();
	AstarNode(const pixel& new_point);
	AstarNode(const pixel& new_point, AstarNode* new_parent);

	getScore();

	private:
	pixel point;
	float G,H;
	AstarNode *parent;

}

using HeuristicFunction = std::function<float(pixel, pixel)>;
using AstarNodeSet = std::vector<AstarNode*>;

class Astar : public SearchAgent
{	
	public:
	Astar(char agent_char, 
		  std::vector<string>& string_list, 
		  std::string& direction_mode,
		  std::string& heuristic_func);

	Astar(char agent_char, 
		  std::vector<string>& string_list, 
		  std::string& direction_mode,
		  int scenarios_number,
		  std::string& heuristic_func);


	friend float CalculateCost(Astar astar_agent, CoordinateList& Path);	

	CoordinateList& GetPath(const int map_index,
							const int par_index,
							float& path_cost,
							double& elapsed_time,
							int& nodes_exp ) const;

	CoordinateList& GetPath(Map& map,
							const pixel& start, 
							const pixel& goal,
							float& path_cost,
							double& elapsed_time,
							int& nodes_exp );

	private:
	string HeuristicMethod;
	AstarNodeSet OpenSet, ClosedSet;
	std::tuple<float,float> cost_scale;




	const float manhattan(const pixel& current, const pixel& target, const float scale);
	const float octile(const pixel& current, const pixel& target, const float scale);
	const float euclidean(const pixel& current, const pixel& target, const float scale);

	void setHeuristic(string& heuristic_func);
	
	int find_index(NodeSet& nodes, Node* curr);
	Node* findNodeOnList(NodeSet& nodes, std::pair<uint, uint>& coordinates);
	std::pair<uint, uint> getDelta(std::pair<uint, uint> source, std::pair<uint, uint> target);

	bool OutOfBounds(Map& map, std::pair<uint, uint>& next);

	bool detectCollision(Node* newNode);

	bool detectCollision(Node& newNode);

	int find_node_coord(int map_x, int map_y, CoordinateList& Path);

	void releaseNodes(NodeSet& nodes);


};



} // namespace GlobalPlanning

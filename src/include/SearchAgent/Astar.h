
#include <functional>
#include <tuple>

#include <glog/logging.h>

#include "search_agent.h"

const std::tuple<float,float,float> default_cost_scale(std::make_tuple(1, sqrt(2), 1);

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


using AstarNodeSet = std::vector<AstarNode*>;
using HeuristicFunction = std::function<int(const pixel&,const pixel&, const float scale)>;

const pixel getDelta(const pixel& current, const pixel& target);
const float manhattan(const pixel& current, const pixel& target, const float scale);
const float octile(const pixel& current, const pixel& target, const float scale);
const float euclidean(const pixel& current, const pixel& target, const float scale);
	
class Astar : public SearchAgent
{	
	public:
	Astar(char agent_char, 
		  std::vector<string>& string_list, 
		  std::string& direction_mode,
		  std::tuple<float,float,float> cost_scale = default_cost_scale,
		  std::string& heuristic_func);

	Astar(char agent_char, 
		  std::vector<string>& string_list, 
		  std::string& direction_mode,
		  int scenarios_number,
		  std::tuple<float,float,float> cost_scale = default_cost_scale,
		  std::string& heuristic_func);


	friend float CalculateCost(Astar astar_agent, CoordinateList& Path);	

	const int GetPath(const int map_index,
				const int par_index,
				CoordinateList& PathFound,
				float& path_cost,
				double& elapsed_time,
				int& nodes_exp ) const;
	
	const int GetPath(Map& map,
				const pixel& start, 
				const pixel& goal,
				CoordinateList& PathFound,
				float& path_cost,
				double& elapsed_time,
				int& nodes_exp ) const;

	private:
	HeuristicFunction heuristic;
	AstarNodeSet OpenSet, ClosedSet;

	/*
       cost scale parameters defined same order as in the tuple:
		0) cartesian direction cost
		1) diagonal direction cost
		2) heuristic distance scale
	*/
	std::tuple<float,float,float> cost_scale;

	void setHeuristic(string& heuristic_method);

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

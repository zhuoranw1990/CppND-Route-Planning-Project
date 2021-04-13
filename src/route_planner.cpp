#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // find the closest nodes to the starting and ending coordinates.
	start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x,end_y);
  	
}

	//h value
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  float distance = node -> distance(*end_node);
  return distance;
}


//expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node -> FindNeighbors();
  for (auto& node : current_node -> neighbors){
    if (node -> visited == false){
      node -> parent = current_node;
      node -> h_value = CalculateHValue(node);
      node -> g_value = current_node -> g_value + current_node -> distance(*node);
      open_list.push_back(node);
      node -> visited = true;
    }
  }
}


//sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
  RouteModel::Node *next_node = nullptr;
  std::sort(open_list.begin(), open_list.end(), [] (const RouteModel::Node* a, const RouteModel::Node* b) {return a->g_value + a->h_value > b->g_value + b->h_value;});
  next_node = open_list.back();
  open_list.pop_back();
  return next_node;
}


//return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
  	while(!(current_node == start_node)){
      path_found.push_back(*current_node);
      distance += current_node -> distance(*(current_node -> parent));
      current_node = current_node -> parent;
    }
  	path_found.emplace_back(*current_node);
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    std::reverse(path_found.begin(), path_found.end());
    return path_found;

}


//A* Search algorithm here.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    current_node = start_node;
  	open_list.push_back(current_node);  
  	current_node -> visited = true;
  	while (!open_list.empty()){     
  	  AddNeighbors(current_node);
      RouteModel::Node *next_node = NextNode();
      if (current_node == end_node){
        m_Model.path = ConstructFinalPath(current_node);
      }     
      current_node = next_node;
    } 
}
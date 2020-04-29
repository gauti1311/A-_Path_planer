#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Done 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    start_node =  &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x,end_y);
}

// Done 3: Implement the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
   return (node->distance(*end_node) );
}


// Done 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(auto neighbor:current_node->neighbors){
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->visited = true;
        open_list.emplace_back(neighbor);
    }
}

// Done 5: Complete the NextNode method to sort the open list and return the next node.
bool LowestSum(RouteModel::Node* node1, RouteModel::Node* node2){
    return (node1->h_value +node1->g_value <node2->h_value +node2->g_value) ;
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(),open_list.end(),LowestSum);
    RouteModel::Node *lowestNode = open_list.front();
    open_list.erase(open_list.begin());
    return lowestNode;

}

// Done 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while (current_node->parent != nullptr)
    {
        path_found.emplace_back(*current_node);
        RouteModel::Node parent = *(current_node->parent);
        distance += current_node->distance(parent);
        current_node = current_node->parent;    
    }
    path_found.emplace_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

// Done 7: Write the A* Search algorithm here.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    start_node->visited = true;
    current_node = start_node;
    // TODO: Implement your solution here.
    while (current_node != nullptr){ 
        
        if(current_node == end_node){ 
          m_Model.path = ConstructFinalPath(current_node);
          return;
        }
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    return;
}
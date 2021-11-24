#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &model.FindClosestNode(start_x,start_y);
    end_node = &model.FindClosestNode(end_x,end_y);

}


// Calculate the heuristic distance of current node to end node.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// Add all unvisited neighbors o fthe current node to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    current_node->FindNeighbors();

    for(int i = 0; i < current_node->neighbors.size(); i++){
        current_node->neighbors[i]->parent = current_node;
        current_node->neighbors[i]->h_value = CalculateHValue(current_node->neighbors[i]);
        current_node->neighbors[i]->g_value = current_node->g_value + current_node->distance(*(current_node->neighbors[i]));
        current_node->neighbors[i]->visited = true;
        open_list.push_back(current_node->neighbors[i]);
    }

}



bool compareFv(RouteModel::Node* a , RouteModel::Node* b){
    float f1 = a->g_value + a->h_value;
    float f2 = b->g_value + b->h_value;
    return f1 > f2;
}
// Sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    
    std::sort(open_list.begin(), open_list.end(), compareFv);

    auto opt = open_list.back();
    open_list.pop_back();

    return opt;

}


// Return the final path found from A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node->parent){
        distance += current_node->distance(*current_node->parent);
        path_found.insert(path_found.begin(), *current_node);
        current_node = current_node->parent;

        if(!(current_node->parent)){
            path_found.insert(path_found.begin(), *current_node);
        }
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// A* search algorithm

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = start_node;
    current_node->visited = true;
    open_list.push_back(current_node);

    while(open_list.size()>0){

        current_node = NextNode();

        if(current_node->x == end_node->x && current_node->y == end_node->y){
            m_Model.path = ConstructFinalPath(end_node);
            return;
        }

        AddNeighbors(current_node);

    }

}
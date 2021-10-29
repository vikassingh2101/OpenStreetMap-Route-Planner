#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(RouteModel::Node *node : current_node->neighbors)
    {
        if( !(node->visited) )
        {
            node->parent = current_node;
            node->g_value = current_node->g_value + current_node->distance(*node);
            node->h_value = CalculateHValue(node);
            node->visited = true;
            open_list.push_back(node);
        }
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    float low_f = open_list[0]->g_value + open_list[0]->h_value;
    RouteModel::Node *node = open_list[0];
    int index = 0;
    for(int i=0; i < open_list.size(); i++)
    {
        float curr_f = open_list[i]->g_value + open_list[i]->h_value;
        if(curr_f < low_f)
        {
            low_f = curr_f;
            node = open_list[i];
            index = i;
        }
    }
    open_list.erase(open_list.begin() + index);
    return node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node *node = current_node;
    while( node != start_node )
    {
        distance += node->distance(*(node->parent));
        path_found.push_back(*node);
        node = node->parent;
    }
    path_found.push_back(*node);

    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = start_node;
    current_node->visited = true;
    while(current_node != end_node)
    {
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    m_Model.path = ConstructFinalPath(end_node);
}
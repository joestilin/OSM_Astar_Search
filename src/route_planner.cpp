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
    for (auto& neighbor_node : current_node->neighbors) {
        neighbor_node->parent = current_node;
        neighbor_node->h_value = CalculateHValue(neighbor_node);
        neighbor_node->g_value = current_node->g_value + neighbor_node->distance(*current_node);
        neighbor_node->visited = true;
        open_list.emplace_back(neighbor_node);
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(this->open_list.begin(), this->open_list.end(),
    [](const auto & n1, const auto & n2) { return n1->h_value + n1->g_value > n2->h_value + n2->g_value;});
    auto node = this->open_list.back();
    this->open_list.pop_back();
    return node;
}

// starting with end_node, traverse the parent nodes backwards to start_node to construct path
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node *next_node = current_node;
    while (next_node != this->start_node) {
        distance += next_node->distance(*(next_node->parent));
        path_found.emplace_back(*next_node);
        next_node = next_node->parent;
    }
    path_found.emplace_back(*next_node);

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


void RoutePlanner::AStarSearch() {

    RouteModel::Node *current_node = nullptr;
    current_node = this->start_node;
    current_node->visited = true;

    this->open_list.emplace_back(current_node);

    while (!open_list.empty()) {
        AddNeighbors(current_node);
        current_node = NextNode();
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
    }
}
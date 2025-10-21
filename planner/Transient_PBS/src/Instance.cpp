#include "Instance.h"

#include <algorithm>  // std::shuffle
#include <chrono>     // std::chrono::system_clock
#include <random>     // std::default_random_engine

// int RANDOM_WALK_STEPS = 100000;

Instance::Instance(shared_ptr<Graph> graph, int screen)
    : graph(graph), screen(screen) {
}

bool Instance::loadAgents(const json &mapf_instance) {
    json starts_json = mapf_instance.at("starts");
    json goals_json = mapf_instance.at("goals");
    this->num_of_agents = starts_json.size();
    this->start_locations.resize(this->num_of_agents);
    this->goal_locations.resize(this->num_of_agents);

    for (int i = 0; i < this->num_of_agents; i++) {
        int start_loc = starts_json[i].at("location");
        int task_loc = goals_json[i].begin()->at("location");
        int task_id = goals_json[i].begin()->at("id");
        this->start_locations[i] = start_loc;
        this->goal_locations[i] = Task(task_id, task_loc);
    }
    return true;
}

void Instance::printAgents() const {
    for (int i = 0; i < num_of_agents; i++) {
        cout << "Agent" << i << " : S=("
             << this->graph->getRowCoordinate(start_locations[i]) << ","
             << this->graph->getColCoordinate(start_locations[i]) << ") ; G=("
             << this->graph->getRowCoordinate(goal_locations[i].loc) << ","
             << this->graph->getColCoordinate(goal_locations[i].loc) << ")"
             << endl;
    }
}

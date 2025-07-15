#include "TaskAssigner.h"

int TaskAssigner::genGoal(set<int> to_avoid, int curr_goal, int start_loc) {
    spdlog::info("Generating a goal for start location {} with curr_goal {}.",
                 start_loc, curr_goal);
    int goal = this->genOneGoalLoc(to_avoid, curr_goal, start_loc);
    spdlog::info("Goal in to avoid: {}", to_avoid.find(goal) != to_avoid.end());
    spdlog::info("Current goal is the same as next goal: {}",
                 goal == curr_goal);
    spdlog::info("Manhattan distance to start location: {}",
                 this->graph->getManhattanDistance(goal, start_loc));
    while (to_avoid.find(goal) != to_avoid.end() || goal == curr_goal ||
           this->graph->getManhattanDistance(goal, start_loc) <
               this->simulation_window) {
        spdlog::info("Goal {} is occupied or too close to start location {}, "
                     "generating a new goal.",
                     goal, start_loc);
        goal = this->genOneGoalLoc(to_avoid, curr_goal, start_loc);
    }
    // Returning
    spdlog::info("Generated goal: {}", goal);
    return goal;
}

int TaskAssigner::genOneGoalLoc(set<int> to_avoid, int curr_goal,
                                int start_loc) {
    int next_goal;
    spdlog::info("Generating a goal location from start location {}.",
                 start_loc);
    // Warehouse tasks alternate between workstations and endpoints
    if (this->graph->warehouse_task_locs.size() > 0) {
        spdlog::info("Generating goal for warehouse tasks.");
        // This is the first goal, determine the next goal based on current
        // location.
        if (curr_goal == -1) {
            spdlog::info("No current goal, generating the first goal.");
            if (this->graph->types[start_loc] == "Workstation") {
                // Print existing endpoints
                // spdlog::info("Current location is a workstation, "
                //              "generating goal from endpoints.");
                // for (int loc : this->graph->endpoints) {
                //     spdlog::info("Endpoint location: {}", loc);
                // }
                next_goal =
                    this->graph
                        ->endpoints[rand() % this->graph->endpoints.size()];
            } else if (this->graph->types[start_loc] == "Endpoint") {
                // // Print existing workstations
                // spdlog::info("Current location is an endpoint, "
                //              "generating goal from workstations.");
                // for (int loc : this->graph->workstations) {
                //     spdlog::info("Workstation location: {}", loc);
                // }
                int rnd = rand() % this->graph->workstations.size();
                spdlog::info("Randomly selected workstation index: {}", rnd);
                spdlog::info("Workstation location: {}",
                             this->graph->workstations[rnd]);
                next_goal = this->graph->workstations[rnd];
            } else {
                // print all warehouse task locations
                // spdlog::info("Current location is not a workstation or "
                //              "endpoint, generating goal from all warehouse "
                //              "task locations.");
                // for (int loc : this->graph->warehouse_task_locs) {
                //     spdlog::info("Warehouse task location: {}", loc);
                // }
                next_goal =
                    this->graph->warehouse_task_locs
                        [rand() % this->graph->warehouse_task_locs.size()];
            }
        }
        // Subsequent goals alternate between workstations and endpoints
        else {
            // If the current goal is a workstation, the next goal is an
            // endpoint
            if (this->graph->types[curr_goal] == "Workstation") {
                next_goal =
                    this->graph
                        ->endpoints[rand() % this->graph->endpoints.size()];
            } else if (this->graph->types[curr_goal] == "Endpoint") {
                next_goal =
                    this->graph->workstations[rand() %
                                              this->graph->workstations.size()];
            } else {
                spdlog::error("Error in genOneGoalLoc: current goal is not a "
                              "workstation or endpoint.");
            }
        }

    }
    // For non-warehouse, generate goals from free locations
    else {
        next_goal =
            this->sampleUnoccupiedLoc(to_avoid, this->graph->free_locations);
    }

    spdlog::info("Generated goal location: {}", next_goal);
    return next_goal;
}

int TaskAssigner::sampleUnoccupiedLoc(set<int> to_avoid,
                                      vector<int> candidates) {
    vector<int> valid_candidates;
    for (int loc : candidates) {
        if (to_avoid.find(loc) == to_avoid.end()) {
            valid_candidates.push_back(loc);
        }
    }

    // Sample from valid candidates
    if (valid_candidates.empty()) {
        spdlog::info("No valid candidates to sample from.");
        return -1;  // or handle the error as needed
    }
    return valid_candidates[rand() % valid_candidates.size()];
}
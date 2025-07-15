#include "TaskAssigner.h"

// Return the goal loc and if the goal is a backup goal.
tuple<int, bool> TaskAssigner::genGoal(set<int> to_avoid, int curr_goal,
                                       int start_loc, int agent_id) {
    // No goal in the buffer, generate a new goal.
    int next_goal;
    if (this->goal_buffer[agent_id] == -1) {
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
                    next_goal = this->graph->sampleEndpoint();
                } else if (this->graph->types[start_loc] == "Endpoint") {
                    next_goal = this->graph->sampleWorkstation();
                } else {
                    next_goal = this->graph->sampleWarehouseTaskLoc();
                }
            }
            // Subsequent goals alternate between workstations and endpoints
            else {
                // If the current goal is a workstation, the next goal is an
                // endpoint
                if (this->graph->types[curr_goal] == "Workstation") {
                    next_goal = this->graph->sampleEndpoint();
                } else if (this->graph->types[curr_goal] == "Endpoint") {
                    next_goal = this->graph->sampleWorkstation();
                } else {
                    spdlog::error(
                        "Agent {}: Must have a goal in the goal buffer.",
                        agent_id);
                }
            }
        }
        // For non-warehouse, generate goals from free locations
        else {
            next_goal = this->graph->sampleFreeLocation();
        }

        // Add goal to buffer. Judge if we want to use it later.
        this->goal_buffer[agent_id] = next_goal;
        // spdlog::info("Generated goal location: {}", next_goal);
        // return next_goal;
    }

    // There is goal in goal buffer, use it if no other agents occupy it.
    next_goal = this->goal_buffer[agent_id];

    // Current goal is occupied, sample a backup goal, i.e. an empty space.
    if (to_avoid.find(next_goal) != to_avoid.end() || next_goal == curr_goal) {
        int backup_goal =
            this->sampleBackupGoal(to_avoid, curr_goal, start_loc, agent_id);
        if (screen > 0) {
            spdlog::info("Agent {}: Current goal {} is occupied, sampling a "
                         "backup goal: {}",
                         agent_id, next_goal, backup_goal);
        }

        return make_tuple(backup_goal, true);
    }

    // If the goal is not occupied, return it and clear the goal buffer.
    this->goal_buffer[agent_id] = -1;  // Clear the goal buffer
    return make_tuple(next_goal, false);
}

// int TaskAssigner::genOneGoalLoc(set<int> to_avoid, int curr_goal, int
// start_loc,
//                                 int agent_id) {
// }

// int TaskAssigner::sampleUnoccupiedLoc(set<int> to_avoid,
//                                       vector<int> candidates) {
//     vector<int> valid_candidates;
//     for (int loc : candidates) {
//         if (to_avoid.find(loc) == to_avoid.end()) {
//             valid_candidates.push_back(loc);
//         }
//     }

//     // Sample from valid candidates
//     if (valid_candidates.empty()) {
//         spdlog::info("No valid candidates to sample from.");
//         return -1;  // or handle the error as needed
//     }
//     return valid_candidates[rand() % valid_candidates.size()];
// }

int TaskAssigner::sampleBackupGoal(set<int> to_avoid, int curr_goal,
                                   int start_loc, int agent_id) {
    int goal = this->graph->sampleEmptyLocation();
    // If the sampled goal is occupied, sample again
    while (to_avoid.find(goal) != to_avoid.end() || goal == curr_goal) {
        goal = this->graph->sampleEmptyLocation();
    }
    return goal;
}
#include "task_assigner.h"

void TaskAssigner::updateGoalLocations(vector<int> start_locations,
                                       set<int> finished_tasks_id) {
    // First goal, randomly choose from all free locations
    // spdlog::info("Update goal locations for {} agents", num_of_agents);
    vector<string> last_goal_types(num_of_agents);
    if (goal_locations.empty()) {
        goal_locations.resize(num_of_agents);
        for (int i = 0; i < num_of_agents; i++) {
            int goal_loc = this->graph->sampleWarehouseTaskLoc();
            goal_locations[i].push_back(
                Task(this->task_id, goal_loc, orient::None));
            this->task_id++;
            last_goal_types[i] = this->graph->types[goal_loc];
        }
    } else {
        // Update the last goal types
        for (int i = 0; i < num_of_agents; i++) {
            last_goal_types[i] =
                this->graph->types[goal_locations[i].back().loc];
        }
    }

    // Remove finished tasks from goal locations
    for (int i = 0; i < num_of_agents; i++) {
        for (int j = 0; j < goal_locations[i].size(); j++) {
            if (finished_tasks_id.find(goal_locations[i][j].id) !=
                finished_tasks_id.end()) {
                goal_locations[i][j].id = -1;  // reset goal id
            }
        }
    }

    // Remove tasks from goal_locations if their id is -1
    for (int i = 0; i < num_of_agents; i++) {
        goal_locations[i].erase(
            remove_if(goal_locations[i].begin(), goal_locations[i].end(),
                      [](const Task& task) { return task.id == -1; }),
            goal_locations[i].end());
    }

    // Generate new goals based on last goal types
    for (int i = 0; i < num_of_agents; i++) {
        // spdlog::info("Agent {}: Last goal type: {}", i, last_goal_types[i]);
        // Minimal time it takes for the robot to reach the available goals
        int min_time = 0;
        int prev_loc = start_locations[i];
        for (const auto& task : goal_locations[i]) {
            min_time += this->graph->getHeuristicOneGoalPebbleMotion(
                            task.loc, prev_loc, orient::None) /
                        V_MAX;
            prev_loc = task.loc;
        }
        // spdlog::info("Agent {}: Minimal time to reach goals: {}", i,
        // min_time);
        while (min_time < this->simulation_window) {
            if (last_goal_types[i] == "Workstation") {
                // If the last goal was a workstation, sample an endpoint
                int new_goal = this->graph->sampleEndpoint();
                if (new_goal != -1) {
                    goal_locations[i].push_back(
                        Task(this->task_id, new_goal, orient::None));
                    this->task_id++;
                    last_goal_types[i] = "Endpoint";
                }
            } else {
                // If the last goal was an endpoint, sample a workstation
                int new_goal = this->graph->sampleWorkstation();
                if (new_goal != -1) {
                    goal_locations[i].push_back(
                        Task(this->task_id, new_goal, orient::None));
                    this->task_id++;
                    last_goal_types[i] = "Workstation";
                }
            }
            min_time +=
                this->graph->getHeuristicOneGoalPebbleMotion(
                    goal_locations[i].back().loc, prev_loc, orient::None) /
                V_MAX;
            prev_loc = goal_locations[i].back().loc;
        }
    }
}

// // Return the goal loc and if the goal is a backup goal.
// tuple<int, bool> TaskAssigner::genGoal(set<int> to_avoid, int curr_goal,
//                                        int start_loc, int agent_id) {
//     // No goal in the buffer, generate a new goal.
//     int next_goal;
//     if (this->goal_buffer[agent_id] == -1) {
//         if (screen > 1) {
//             spdlog::info("Generating a goal location from start location
//             {}.",
//                          start_loc);
//         }

//         // Warehouse tasks alternate between workstations and endpoints
//         if (this->graph->warehouse_task_locs.size() > 0) {
//             if (screen > 1)
//                 spdlog::info("Generating goal for warehouse tasks.");
//             // This is the first goal, determine the next goal based on
//             current
//             // location.
//             if (curr_goal == -1) {
//                 if (screen > 1)
//                     spdlog::info("No current goal, generating the first
//                     goal.");
//                 if (this->graph->types[start_loc] == "Workstation") {
//                     next_goal = this->sampleUnoccupiedLoc(
//                         to_avoid, this->graph->endpoints);
//                 } else if (this->graph->types[start_loc] == "Endpoint") {
//                     next_goal = this->sampleUnoccupiedLoc(
//                         to_avoid, this->graph->workstations);
//                 } else {
//                     next_goal = this->sampleUnoccupiedLoc(
//                         to_avoid, this->graph->warehouse_task_locs);
//                 }
//             }
//             // Subsequent goals alternate between workstations and endpoints
//             else {
//                 // If the current goal is a workstation, the next goal is an
//                 // endpoint
//                 if (this->graph->types[curr_goal] == "Workstation") {
//                     next_goal = this->sampleUnoccupiedLoc(
//                         to_avoid, this->graph->endpoints);
//                 } else if (this->graph->types[curr_goal] == "Endpoint") {
//                     next_goal = this->sampleUnoccupiedLoc(
//                         to_avoid, this->graph->workstations);
//                 } else {
//                     spdlog::error(
//                         "Agent {}: Must have a goal in the goal buffer.",
//                         agent_id);
//                     exit(-1);
//                     // The agent fails to sample a goal last time, try again
//                     // here.
//                     // next_goal = this->sampleUnoccupiedLoc(
//                     //     to_avoid, this->graph->warehouse_task_locs);
//                 }
//             }
//         }
//         // For non-warehouse, generate goals from free locations
//         else {
//             next_goal = this->sampleUnoccupiedLoc(to_avoid,
//                                                   this->graph->free_locations);
//         }

//         // if (next_goal == -1) {
//         //     if (screen > 0) {
//         //         spdlog::info(
//         //             "Agent {}: No valid goal found from start location {},
//         "
//         //             "sampling a backup goal.",
//         //             agent_id, start_loc);
//         //     }
//         //     int backup_goal = this->sampleBackupGoal(to_avoid, curr_goal,
//         //                                              start_loc, agent_id);
//         //     return make_tuple(backup_goal, false);  // No valid goal found
//         // }

//         // Add goal to buffer. Judge if we want to use it later.
//         this->goal_buffer[agent_id] = next_goal;
//         // spdlog::info("Generated goal location: {}", next_goal);
//         // return next_goal;
//     }

//     // If the goal is not occupied, return it and clear the goal buffer.
//     this->goal_buffer[agent_id] = -1;  // Clear the goal buffer
//     return make_tuple(next_goal, false);
// }
#include "task_assigners/DistinctOneGoalTaskAssigner.h"

DistinctOneGoalTaskAssigner::DistinctOneGoalTaskAssigner(
    const SMARTGrid &G, const shared_ptr<HeuristicTableBase> heuristic_table,
    int screen, int num_of_agents, int seed, string task_file)
    : BasicTaskAssigner(G, heuristic_table, screen, num_of_agents, seed,
                        task_file) {
    this->goal_buffer.resize(num_of_agents, -1);

    // Each agent has only one goal at a time
    for (int i = 0; i < num_of_agents; i++) {
        this->goal_locations[i].resize(1);
        this->goal_locations[i][0] = Task();
        this->goal_locations[i][0].id = -1;  // no goal assigned
    }
}

void DistinctOneGoalTaskAssigner::updateStartsAndGoals(
    vector<tuple<double, double, int>> &start_locs,
    set<int> finished_tasks_id) {
    // Set new starts
    for (int i = 0; i < this->num_of_agents; i++) {
        // Obtain the starts
        int row = static_cast<int>(std::get<0>(start_locs[i]));
        int col = static_cast<int>(std::get<1>(start_locs[i]));
        int ori = std::get<2>(start_locs[i]);
        this->starts[i] = State(this->G.getCellId(row, col), 0,
                                this->G.ORI_SMART_TO_RHCR.at(ori));
    }

    // Remove finished tasks from goal locations
    set<int> unfinished_goal_locs;  // For duplicate checking
    for (int i = 0; i < num_of_agents; i++) {
        if (finished_tasks_id.find(goal_locations[i].begin()->id) !=
            finished_tasks_id.end()) {
            goal_locations[i].begin()->id = -1;  // reset goal id
        } else {
            unfinished_goal_locs.insert(goal_locations[i].begin()->location);
        }
    }

    // Generate new goals
    for (int i = 0; i < num_of_agents; i++) {
        // generate a goal for the agent if it does not have one
        if (goal_locations[i].begin()->id == -1) {
            if (screen > 1)
                spdlog::info("Agent {} has no goal, generating a new one.", i);
            int curr_goal = goal_locations[i].begin()->location;
            int next_goal;
            bool back_up;
            tie(next_goal, back_up) = this->genGoal(
                unfinished_goal_locs, curr_goal, starts[i].location, i);
            goal_locations[i][0] =
                Task(next_goal, -1, 0, 0, i, -1, false, false, this->task_id);
            if (back_up)
                this->backup_tasks.insert(goal_locations[i].begin()->id);
            unfinished_goal_locs.insert(goal_locations[i].begin()->location);
            this->task_id++;
        }
    }
}

// Return the goal loc and if the goal is a backup goal.
tuple<int, bool> DistinctOneGoalTaskAssigner::genGoal(set<int> to_avoid,
                                                      int curr_goal,
                                                      int start_loc,
                                                      int agent_id) {
    // No goal in the buffer, generate a new goal.
    int next_goal;
    if (this->goal_buffer[agent_id] == -1) {
        if (screen > 1) {
            spdlog::info("Generating a goal location from start location {}.",
                         start_loc);
        }

        // Warehouse tasks alternate between workstations and endpoints
        if (this->G.workstations.size() > 0 && this->G.endpoints.size() > 0) {
            if (screen > 1)
                spdlog::info("Generating goal for warehouse tasks.");
            // This is the first goal, determine the next goal based on current
            // location.
            if (curr_goal == -1) {
                if (screen > 1)
                    spdlog::info("No current goal, generating the first goal.");
                if (this->G.types[start_loc] == "Workstation") {
                    next_goal =
                        this->sampleUnoccupiedLoc(to_avoid, this->G.endpoints);
                } else if (this->G.types[start_loc] == "Endpoint") {
                    next_goal = this->sampleUnoccupiedLoc(to_avoid,
                                                          this->G.workstations);
                } else {
                    next_goal = this->sampleUnoccupiedLoc(
                        to_avoid, this->G.task_locations);
                }
            }
            // Subsequent goals alternate between workstations and endpoints
            else {
                // If the current goal is a workstation, the next goal is an
                // endpoint
                if (this->G.types[curr_goal] == "Workstation") {
                    next_goal =
                        this->sampleUnoccupiedLoc(to_avoid, this->G.endpoints);
                } else if (this->G.types[curr_goal] == "Endpoint") {
                    next_goal = this->sampleUnoccupiedLoc(to_avoid,
                                                          this->G.workstations);
                } else {
                    spdlog::error(
                        "Agent {}: Must have a goal in the goal buffer.",
                        agent_id);
                    exit(-1);
                }
            }
        }
        // For non-warehouse, generate goals from free locations
        else {
            next_goal =
                this->sampleUnoccupiedLoc(to_avoid, this->G.free_locations);
        }

        // Add goal to buffer. Judge if we want to use it later.
        this->goal_buffer[agent_id] = next_goal;
    }

    // There is goal in goal buffer, use it if no other agents occupy it.
    next_goal = this->goal_buffer[agent_id];

    // Current goal is occupied, sample a backup goal, i.e. an empty space.
    if (to_avoid.find(next_goal) != to_avoid.end() || next_goal == curr_goal) {
        int backup_goal =
            this->sampleBackupGoal(to_avoid, curr_goal, start_loc, agent_id);
        if (screen > 1) {
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

int DistinctOneGoalTaskAssigner::sampleUnoccupiedLoc(set<int> to_avoid,
                                                     vector<int> candidates) {
    vector<int> valid_candidates;
    for (int loc : candidates) {
        if (to_avoid.find(loc) == to_avoid.end()) {
            valid_candidates.push_back(loc);
        }
    }

    // Sample from valid candidates
    if (valid_candidates.empty()) {
        if (screen > 1) {
            spdlog::info(
                "No valid candidates to sample from. Sampling from all "
                "candidates.");
        }

        // We can only sample from the candidate. The tasking logic later will
        // prevent the agent from going to there momentarily.
        return candidates[rand() % candidates.size()];
    }
    return valid_candidates[rand() % valid_candidates.size()];
}

int DistinctOneGoalTaskAssigner::sampleBackupGoal(set<int> to_avoid,
                                                  int curr_goal, int start_loc,
                                                  int agent_id) {
    // According to the heuristic, select the nearest unoccupied free location
    // as the backup goal.
    double min_h_val = WEIGHT_MAX;
    int best_loc = -1;
    for (int loc : this->G.free_locations) {
        if (to_avoid.find(loc) == to_avoid.end() && loc != curr_goal) {
            // Compute heuristic
            double h_val = this->heuristic_table->get(loc, start_loc);
            if (h_val < min_h_val) {
                min_h_val = h_val;
                best_loc = loc;
            }
        }
    }

    if (best_loc == -1) {
        spdlog::error("Agent {}: No valid backup goal found.", agent_id);
        // int goal = this->G.free_locations[rand() %
        // this->G.free_locations.size()];
        // // If the sampled goal is occupied, sample again
        // while (to_avoid.find(goal) != to_avoid.end() || goal == curr_goal) {
        //     goal = this->G.free_locations[rand() %
        //     this->G.free_locations.size()];
        // }
    }

    return best_loc;
}
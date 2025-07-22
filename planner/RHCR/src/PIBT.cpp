#include "PIBT.h"

PIBT::PIBT(const BasicGraph &G, SingleAgentSolver &path_planner)
    : MAPFSolver(G, path_planner) {
}

bool PIBT::run(const vector<State> &starts,
               const vector<vector<Task>> &goal_locations, int time_limit,
               const vector<int> &waited_time) {
    // set timer
    clock_t start = std::clock();
    num_expanded = 0;
    num_generated = 0;
    num_restarts = 0;
    int num_of_agents = starts.size();

    // Initialize
    this->curr_occupied.clear();
    this->curr_occupied.resize(this->G.size(), -1);
    this->next_occupied.clear();
    this->next_occupied.resize(this->G.size(), -1);

    this->is_waiting.clear();
    this->is_waiting.resize(num_of_agents, false);

    vector<int> curr_waited_time(num_of_agents, 0);

    if (solution.size() == 0)
        solution.resize(num_of_agents);

    // `goals_mem` is a memory of goals for each agent. Reached goals will be
    // removed.
    this->goals_mem = goal_locations;

    // Pre-process solutions
    for (int i = 0; i < num_of_agents; i++) {
        // Clear previous solutions, if any
        solution[i].clear();
        solution[i].resize(this->simulation_window + 1);
        // The agent starts at timestep 0, initialize the solution with the
        // start state
        if (starts[i].timestep == 0 &&
            starts[i].location != goal_locations[i][0].location) {
            solution[i][0] = starts[i];
            // For now PIBT only works with no orientation.
            // TODO: Add support for orientation
            solution[i][0].orientation = -1;
            this->curr_occupied[starts[i].location] = i;
        }
        // The agent does not start at timestep 0, or the first goal is right at
        // the its initial timestep, then it is waiting at the start until the
        // given timestep
        else {
            int n_t_to_wait = 0;
            if (starts[i].timestep > 0) {
                n_t_to_wait = starts[i].timestep + 1;
            } else {
                this->goals_mem[i].erase(this->goals_mem[i].begin());
                n_t_to_wait = goal_locations[i][0].task_wait_time + 1;
            }

            State actual_start = State(starts[i]);
            actual_start.timestep = 0;
            actual_start.orientation = -1;  // No orientation in PIBT
            actual_start.is_tasking_wait = true;
            solution[i][0] = actual_start;
            int t = 1;
            while (t < n_t_to_wait && t < this->simulation_window + 1) {
                solution[i][t] = solution[i][t - 1].wait();
                t++;
            }
            // is_waiting[i] = false;
            // curr_waited_time[i] += starts[i].timestep;
            this->curr_occupied[starts[i].location] = i;
            this->next_occupied[starts[i].location] = i;
        }
    }

    // Print the solution
    if (screen > 1) {
        cout << "Initialized solution:" << endl;
        for (int i = 0; i < num_of_agents; i++) {
            cout << "Agent " << i << ": ";
            for (const auto &state : solution[i]) {
                cout << "(" << state.location << "," << state.timestep << ","
                     << state.orientation << "," << state.is_tasking_wait
                     << ") ";
            }
            cout << endl;
        }
    }

    // Order of agents
    vector<int> agents(num_of_agents);
    for (int i = 0; i < num_of_agents; i++) {
        agents[i] = i;
    }

    for (int t = 1; t < this->simulation_window + 1; t++) {
        // Each iteration plans from t - 1 to t, until t is larer than
        // planning_window

        // if (screen > 1) {
        //     cout << "#################" << endl;
        //     cout << "Planning from timestep " << t - 1 << " to " << t <<
        //     endl;
        //     // Print curr occupied and next occupied locations
        //     cout << "Current occupied locations: ";
        //     for (int i = 0; i < this->curr_occupied.size(); i++) {
        //         if (this->curr_occupied[i] != -1) {
        //             cout << i << "(" << this->curr_occupied[i] << ") ";
        //         }
        //     }
        //     cout << endl;
        //     cout << "Next occupied locations: ";
        //     for (int i = 0; i < this->next_occupied.size(); i++) {
        //         if (this->next_occupied[i] != -1) {
        //             cout << i << "(" << this->next_occupied[i] << ") ";
        //         }
        //     }
        // }

        // Planning
        // Sort agents by their distance to the goal, breaking ties randomly.
        auto agents_cmp = [&](int a, int b) {
            // In one bot per aisle grid, we prioritize the agents that are
            // deeper in the aisles
            int loc_a = solution[a][t - 1].location;
            int loc_b = solution[b][t - 1].location;
            // if (this->G.get_grid_type() == SMARTGridType::ONE_BOT_PER_AISLE &&
            //     this->G.getColCoordinate(loc_a) ==
            //         this->G.getColCoordinate(loc_b)) {
            //     {
            //         if (this->G.types[loc_a] == this->G.types[loc_b]) {
            //             if (this->G.types[loc_a] == "Endpoint")
            //                 return this->G.getRowCoordinate(loc_a) <
            //                        this->G.getRowCoordinate(loc_b);
            //             else if (this->G.types[loc_a] == "Workstation")
            //                 return this->G.getColCoordinate(loc_a) >
            //                        this->G.getColCoordinate(loc_b);
            //         }
            //         else {
            //             // if the types are different, we prioritize the agent
            //             // in the workstation or endpoint
            //             return this->G.types[loc_a] == "Workstation" ||
            //                    this->G.types[loc_a] == "Endpoint";
            //         }
            //     }
            // }

            // Compare the distance to the goal for each agent
            // If one of the agents has no goals, do not compare
            if (this->goals_mem[a].empty() || this->goals_mem[b].empty())
                return false;
            double dist_a = this->G.get_heuristic(
                this->goals_mem[a][0].location, solution[a][t - 1].location,
                solution[a][t - 1].orientation);
            double dist_b = this->G.get_heuristic(
                this->goals_mem[b][0].location, solution[b][t - 1].location,
                solution[b][t - 1].orientation);
            // Prefer the agent with smaller distance to its goal
            if (dist_a != dist_b)
                return dist_a < dist_b;
            // If the distances are equal, break ties randomly
            return false;
        };
        std::shuffle(agents.begin(), agents.end(), this->gen);
        std::sort(agents.begin(), agents.end(), agents_cmp);
        for (int k : agents) {
            // If the agent is already planned, skip it
            if (solution[k][t].location == -1 &&
                this->goals_mem[k].size() > 0) {
                // Determine the start loc and goals
                State start = solution[k][t - 1];
                if (this->goals_mem[k].empty()) {
                    cout << "No goals for agent " << k << endl;
                    continue;  // No goals left for this agent
                }
                vector<Task> goals = this->goals_mem[k];
                if (screen > 1) {
                    cout << "Planning agent " << k << " from " << start.location
                         << " at timestep " << start.timestep
                         << " with goals: ";
                    for (const auto &goal : goals) {
                        cout << goal.location << " ";
                    }
                    cout << endl;
                }
                bool success = pibt_funct(k, -1, start, goals[0], t - 1);
            }
        }

        // Acting
        for (int k = 0; k < num_of_agents; k++) {
            // Update the next occupied locations
            this->next_occupied[solution[k][t].location] = -1;

            // Update the current occupied location
            if (this->curr_occupied[solution[k][t - 1].location] == k) {
                // The agent is still occupying the location
                this->curr_occupied[solution[k][t - 1].location] = -1;
            }
            this->curr_occupied[solution[k][t].location] = k;

            // Check for any goals reached
            State state = solution[k][t];
            auto curr_goal = this->goals_mem[k].begin();
            if (!this->goals_mem[k].empty() &&
                state.location == curr_goal->location) {
                // Agent is at the goal, and it finished waiting or does not
                // need to wait
                if (curr_waited_time[k] >= curr_goal->task_wait_time) {
                    this->goals_mem[k].erase(curr_goal);
                    curr_waited_time[k] = 0;  // Reset the waited time
                    // The agent is not waiting anymore
                    this->is_waiting[k] = false;
                } else {
                    // The agent requires waiting
                    this->is_waiting[k] = true;
                    curr_waited_time[k] += 1;
                }
            }

            // Some agent might already has path in the next timestep (such
            // as those tasking wait agents). We need to update the curr and
            // next occupied locations accordingly.
            if (t + 1 < this->simulation_window + 1 &&
                solution[k][t + 1].location != -1) {
                // The agent has a path in the next timestep
                this->next_occupied[solution[k][t + 1].location] = k;
            }
        }

        // Print the content of goal_mem:
        if (screen > 1) {
            cout << "Current solution:" << endl;
            for (int i = 0; i < num_of_agents; i++) {
                cout << "Agent " << i << ": ";
                for (const auto &state : solution[i]) {
                    cout << "(" << state.location << "," << state.timestep
                         << "," << state.orientation << ","
                         << state.is_tasking_wait << ") ";
                }
                cout << endl;
            }
            cout << "Goals memory after planning from " << t - 1 << " to " << t
                 << ": " << endl;
            for (int i = 0; i < num_of_agents; i++) {
                cout << "Agent " << i << ": ";
                for (const auto &goal : this->goals_mem[i]) {
                    cout << goal.location << " ";
                }
                cout << endl;
            }
            cout << "End planning from timestep " << t - 1 << " to " << t
                 << endl;
            cout << "#################" << endl;
        }
    }
    if (screen > 0)
        print_results();
    return true;
}

bool PIBT::pibt_funct(int a_i, int a_j, State start_state, Task goal,
                      int from_t) {
    // Determine the actions
    // State tmp_start(start_state);
    // // Use a dummy start state to get the movement neighbors
    // tmp_start.orientation = -1;
    // list<State> next_states_ = this->G.get_neighbors(tmp_start);

    // if (start_state.orientation >= 0) {
    //     for (auto &next_state : next_states_) {
    //         // Get the orientation at the next state
    //         int next_ori = this->G.get_direction(start_state.location,
    //                                              next_state.location);

    //         // Set the orientation of the next state
    //         if (next_ori < 4)
    //             next_state.orientation = next_ori;
    //         else
    //             next_state.orientation = start_state.orientation;
    //     }
    // }

    // vector<State> next_states(next_states_.begin(), next_states_.end());

    // Sort the next states by action cost + heuristic values, breaking ties
    // randomly.
    // auto action_cmp = [&](const State &n1, const State &n2) {
    //     // Rotation cost is the cost of rotating
    //     double rot_cost1 = 0;
    //     double rot_cost2 = 0;
    //     if (start_state.orientation >= 0) {
    //         // Get the rotation degree
    //         int rot_degree1 =
    //         this->G.get_rotate_degree(start_state.orientation,
    //                                                     n1.orientation);
    //         int rot_degree2 =
    //         this->G.get_rotate_degree(start_state.orientation,
    //                                                     n2.orientation);

    //         // Calculate the rotation cost
    //         double cost1 =
    //             this->G.get_weight(start_state.location, n1.location);
    //         double cost2 =
    //             this->G.get_weight(start_state.location, n2.location);
    //         rot_cost1 = rot_degree1 * cost1;
    //         rot_cost2 = rot_degree2 * cost2;
    //     }

    //     // Movement cost is the cost of moving from start loc to end loc
    //     double move_cost1 =
    //         this->G.get_weight(start_state.location, n1.location);
    //     double move_cost2 =
    //         this->G.get_weight(start_state.location, n2.location);
    //     double h1 =
    //         this->G.get_heuristic(goal.location, n1.location,
    //         n1.orientation);
    //     double h2 =
    //         this->G.get_heuristic(goal.location, n2.location,
    //         n2.orientation);
    //     double cost1 = move_cost1 + rot_cost1 + h1;
    //     double cost2 = move_cost2 + rot_cost2 + h2;
    //     if (cost1 != cost2)
    //         return cost1 < cost2;
    //     // Tie breaking. Prefer the next state that is currently not
    //     occupied if (this->curr_occupied[n1.location] == -1 &&
    //         this->curr_occupied[n2.location] != -1) {
    //         return true;  // n1 is not occupied, n2 is occupied
    //     } else if (this->curr_occupied[n1.location] != -1 &&
    //                this->curr_occupied[n2.location] == -1) {
    //         return false;  // n2 is not occupied, n1 is occupied
    //     }
    //     return false;
    // };

    // ########### OLD implementation without orientation ###########
    // Determine the actions
    list<State> next_states_ = this->G.get_neighbors(start_state);

    // If the agent is waiting, it can only wait in place
    if (this->is_waiting[a_i]) {
        // If the agent is waiting, it can only wait in place
        next_states_.clear();
        next_states_.push_back(start_state.wait());
    }

    // Print next states
    // if (screen > 1) {
    //     cout << "Next states for agent " << a_i << " at timestep " <<
    //     from_t
    //          << ": " << endl;
    //     for (const auto &next_state : next_states_) {
    //         cout << next_state << " (cost: "
    //              << this->G.get_weight(start_state.location,
    //                                    next_state.location)
    //              << ", heuristic: "
    //              << this->G.get_heuristic(goal.location,
    //              next_state.location,
    //                                       next_state.orientation)
    //              << ")" << endl;
    //     }
    //     cout << endl;
    // }

    vector<State> next_states(next_states_.begin(), next_states_.end());

    // Sort the next states by action cost + heuristic values, breaking ties
    // randomly.
    auto action_cmp = [&](const State &n1, const State &n2) {
        double cost1 = this->G.get_weight(start_state.location, n1.location);
        double cost2 = this->G.get_weight(start_state.location, n2.location);
        double h1 =
            this->G.get_heuristic(goal.location, n1.location, n1.orientation);
        double h2 =
            this->G.get_heuristic(goal.location, n2.location, n2.orientation);
        if (cost1 + h1 != cost2 + h2)
            return (cost1 + h1) < (cost2 + h2);
        // Tie breaking. Prefer the next state that is currently not
        // occupied
        if (this->curr_occupied[n1.location] == -1 &&
            this->curr_occupied[n2.location] != -1) {
            return true;  // n1 is not occupied, n2 is occupied
        } else if (this->curr_occupied[n1.location] != -1 &&
                   this->curr_occupied[n2.location] == -1) {
            return false;  // n2 is not occupied, n1 is occupied
        }
        return false;
    };
    // ########### OLD implementation without orientation ###########

    // Shuffle the next states to break ties randomly, then sort
    std::shuffle(next_states.begin(), next_states.end(), this->gen);
    std::sort(next_states.begin(), next_states.end(), action_cmp);

    // cout << "start state: " << start_state << endl;

    // cout << "Next states: " << endl;
    // for (const auto &next_state : next_states) {
    //     cout << next_state << " (cost: "
    //          << this->G.get_weight(start_state.location,
    //          next_state.location)
    //          << ", heuristic: "
    //          << this->G.get_heuristic(goal.location, next_state.location,
    //                                   next_state.orientation)
    //          << ")" << endl;
    // }
    // cout << endl;

    for (auto next_state : next_states) {
        // // For rotation, infer the actual next state
        // if (start_state.orientation >= 0 &&
        //     start_state.location != next_state.location &&
        //     start_state.orientation != next_state.orientation) {
        //     // The agent is rotating and moving at the same time. We make
        //     it
        //     // rotate first
        //     cout << "Moving and rotating from " << start_state << " to "
        //          << next_state << endl;
        //     State actual_next_state(next_state);
        //     actual_next_state.location = start_state.location;

        //     // Check if the rotation is 180 degrees
        //     if (this->G.get_rotate_degree(start_state.orientation,
        //                                   next_state.orientation) >= 2) {
        //         // Always turn left
        //         actual_next_state.orientation =
        //             (start_state.orientation + 1) % 4;
        //         cout << "Rotating 90 degrees from " <<
        //         start_state.orientation
        //              << " to " << actual_next_state.orientation << endl;
        //     }

        //     cout << "Actual next state: " << actual_next_state << endl;
        //     cout << endl;

        //     next_state = actual_next_state;
        // }

        // avoid vertex conflicts
        if (this->next_occupied[next_state.location] != -1) {
            continue;
        }

        // avoid edge conflicts
        if (a_j != -1 &&
            next_state.location == solution[a_j][from_t].location) {
            continue;
        }

        // reserve the next state
        this->next_occupied[next_state.location] = a_i;
        this->solution[a_i][from_t + 1] = next_state;

        // Another agent a_k is currently occupying the next state. Replan
        // it, if it is not planned.
        int a_k = curr_occupied[next_state.location];
        if (a_k != -1 && solution[a_k][from_t + 1].location == -1 &&
            this->goals_mem[a_k].size() > 0) {
            if (!pibt_funct(a_k, a_i, solution[a_k][from_t],
                            this->goals_mem[a_k][0], from_t)) {
                continue;
            }
        }
        return true;
    }

    // Fail to find a valid next state, we can only let a_i wait in place.
    next_occupied[start_state.location] = a_i;
    State wait = start_state.wait();
    wait.is_tasking_wait = false;
    this->solution[a_i][from_t + 1] = wait;
    return false;
}

void PIBT::print_results() const {
    // std::cout << "PIBT*:Succeed," << runtime << "," << std::endl;
    string result_str = "PIBT*:Succeed,Runtime=" + to_string(runtime);
    spdlog::info(result_str);
}

void PIBT::save_results(const std::string &fileName,
                        const std::string &instanceName) const {
    std::ofstream stats;
    stats.open(fileName, std::ios::app);
    stats << runtime << "," << num_restarts << "," << num_restarts << ","
          << num_expanded << "," << num_generated << "," << solution_cost << ","
          << min_sum_of_costs << "," << avg_path_length << "," << "0"
          << "," << instanceName << std::endl;
    stats.close();
}

void PIBT::clear() {
    runtime = 0;
    solution_found = false;
    solution_cost = -2;
    avg_path_length = -1;
    num_expanded = 0;
    num_generated = 0;
    num_restarts = 0;
    solution.clear();
    initial_constraints.clear();
}
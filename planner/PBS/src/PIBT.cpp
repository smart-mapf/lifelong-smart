#include "PIBT.h"

bool PIBT::run(const vector<State> &starts,
               const vector<Task> &goal_locations) {
    // set timer
    clock_t start = std::clock();
    // int num_agents = starts.size();
    this->goal_locations = goal_locations;

    // Initialize
    this->curr_occupied.resize(this->G->size(), -1);
    this->next_occupied.resize(this->G->size(), -1);

    if (solution.size() == 0)
        solution.resize(num_agents);

    // Clear previous solutions, if any
    for (int i = 0; i < num_agents; i++) {
        solution[i].clear();
        solution[i].resize(this->simulation_window);
        // Initialize the solution with the start state
        solution[i][0] = starts[i];
        // For now PIBT only works with no orientation.
        // TODO: Add support for orientation
        solution[i][0].orientation = -1;
        this->curr_occupied[starts[i].location] = i;
    }

    // Order of agents
    vector<int> agents(num_agents);
    for (int i = 0; i < num_agents; i++) {
        agents[i] = i;
    }

    for (int t = 1; t < this->simulation_window; t++) {
        // Each iteration plans from t - 1 to t, until t is larer than
        // planning_window

        if (screen > 1) {
            cout << "#################" << endl;
            cout << "Planning from timestep " << t - 1 << " to " << t << endl;
        }

        // Planning
        // Sort agents by their distance to the goal, breaking ties randomly.
        auto agents_cmp = [&](int a, int b) {
            // Compare the distance to the goal for each agent
            double dist_a = this->G->getHeuristic(goal_locations[a].loc,
                                                  solution[a][t - 1].location);
            double dist_b = this->G->getHeuristic(goal_locations[b].loc,
                                                  solution[b][t - 1].location);
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
            if (solution[k][t].location == -1) {
                // Determine the start loc and goals
                State start = solution[k][t - 1];
                Task goal = goal_locations[k];
                if (screen > 1) {
                    cout << "Planning agent " << k << " from " << start.location
                         << " to " << goal.loc << " at timestep " << t - 1
                         << endl;
                }
                bool success = pibt_funct(k, -1, start, goal, t - 1);
            }
        }

        // Acting
        for (int k = 0; k < num_agents; k++) {
            // Update the next occupied locations
            this->next_occupied[solution[k][t].location] = -1;

            // Update the current occupied location
            if (this->curr_occupied[solution[k][t - 1].location] == k) {
                // The agent is still occupying the location
                this->curr_occupied[solution[k][t - 1].location] = -1;
            }
            this->curr_occupied[solution[k][t].location] = k;

            // // Check for any goals reached
            // State state = solution[k][t];
            // if (!goal_locations[k].empty() &&
            //     state.location == goal_locations[k].begin()->location) {
            //     goal_locations[k].erase(goal_locations[k].begin());
            // }
        }

        // // Print the content of goal_mem:
        // if (screen > 1) {
        //     cout << "Goals memory after planning from " << t - 1 << " to " <<
        //     t
        //          << ": " << endl;
        //     for (int i = 0; i < num_agents; i++) {
        //         cout << "Agent " << i << ": ";
        //         for (const auto &goal : goal_locations[i]) {
        //             cout << goal.location << " ";
        //         }
        //         cout << endl;
        //     }
        //     cout << "End planning from timestep " << t - 1 << " to " << t
        //          << endl;
        //     cout << "#################" << endl;
        // }
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
    // list<State> next_states_ = this->G->get_neighbors(tmp_start);

    // if (start_state.orientation >= 0) {
    //     for (auto &next_state : next_states_) {
    //         // Get the orientation at the next state
    //         int next_ori = this->G->get_direction(start_state.location,
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
    //         this->G->get_rotate_degree(start_state.orientation,
    //                                                     n1.orientation);
    //         int rot_degree2 =
    //         this->G->get_rotate_degree(start_state.orientation,
    //                                                     n2.orientation);

    //         // Calculate the rotation cost
    //         double cost1 =
    //             this->G->get_weight(start_state.location, n1.location);
    //         double cost2 =
    //             this->G->get_weight(start_state.location, n2.location);
    //         rot_cost1 = rot_degree1 * cost1;
    //         rot_cost2 = rot_degree2 * cost2;
    //     }

    //     // Movement cost is the cost of moving from start loc to end loc
    //     double move_cost1 =
    //         this->G->get_weight(start_state.location, n1.location);
    //     double move_cost2 =
    //         this->G->get_weight(start_state.location, n2.location);
    //     double h1 =
    //         this->G->getHeuristic(goal.location, n1.location,
    //         n1.orientation);
    //     double h2 =
    //         this->G->getHeuristic(goal.location, n2.location,
    //         n2.orientation);
    //     double cost1 = move_cost1 + rot_cost1 + h1;
    //     double cost2 = move_cost2 + rot_cost2 + h2;
    //     if (cost1 != cost2)
    //         return cost1 < cost2;
    //     // Tie breaking. Prefer the next state that is currently not occupied
    //     if (this->curr_occupied[n1.location] == -1 &&
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
    list<int> next_states_ = this->G->getNeighbors(start_state.location);
    vector<int> next_states(next_states_.begin(), next_states_.end());

    // Sort the next states by action cost + heuristic values, breaking ties
    // randomly.
    auto action_cmp = [&](const State &n1, const State &n2) {
        double cost1 = this->G->getWeight(start_state.location, n1.location);
        double cost2 = this->G->getWeight(start_state.location, n2.location);
        double h1 = this->G->getHeuristic(goal.loc, n1.location);
        double h2 = this->G->getHeuristic(goal.loc, n2.location);
        if (cost1 + h1 != cost2 + h2)
            return (cost1 + h1) < (cost2 + h2);
        // Tie breaking. Prefer the next state that is currently not occupied
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
    //          << this->G->get_weight(start_state.location,
    //          next_state.location)
    //          << ", heuristic: "
    //          << this->G->getHeuristic(goal.location, next_state.location,
    //                                   next_state.orientation)
    //          << ")" << endl;
    // }
    // cout << endl;

    for (auto next_state : next_states) {
        // // For rotation, infer the actual next state
        // if (start_state.orientation >= 0 &&
        //     start_state.location != next_state.location &&
        //     start_state.orientation != next_state.orientation) {
        //     // The agent is rotating and moving at the same time. We make it
        //     // rotate first
        //     cout << "Moving and rotating from " << start_state << " to "
        //          << next_state << endl;
        //     State actual_next_state(next_state);
        //     actual_next_state.location = start_state.location;

        //     // Check if the rotation is 180 degrees
        //     if (this->G->get_rotate_degree(start_state.orientation,
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
        if (this->next_occupied[next_state] != -1)
            continue;
        // avoid edge conflicts
        if (a_j != -1 && next_state == solution[a_j][from_t].location)
            continue;

        // reserve the next state
        this->next_occupied[next_state] = a_i;
        this->solution[a_i][from_t + 1] = next_state;

        // Another agent a_k is currently occupying the next state. Replan it,
        // if it is not planned.
        int a_k = curr_occupied[next_state];
        if (a_k != -1 && solution[a_k][from_t + 1].location == -1) {
            if (!pibt_funct(a_k, a_i, solution[a_k][from_t],
                            goal_locations[a_k], from_t)) {
                continue;
            }
        }
        return true;
    }

    // Fail to find a valid next state, we can only let a_i wait in place.
    next_occupied[start_state.location] = a_i;
    this->solution[a_i][from_t + 1] = start_state.wait();
    return false;
}

void PIBT::print_results() const {
    // std::cout << "PIBT*:Succeed," << runtime << "," << std::endl;
    // string result_str = "PIBT*:Succeed,Runtime=" + to_string(runtime);
    spdlog::info("PIBT:Succeed,Runtime={}", runtime);
}

void PIBT::save_results(const std::string &fileName,
                        const std::string &instanceName) const {
    std::ofstream stats;
    stats.open(fileName, std::ios::app);
    stats << runtime << "," << std::endl;
    stats.close();
}

void PIBT::clear() {
    runtime = 0;
    solution_found = false;
    solution.clear();
}

vector<vector<tuple<int, int, double, int>>> PIBT::getPaths() {
    std::vector<std::vector<std::tuple<int, int, double, int>>> new_mapf_plan;
    new_mapf_plan.resize(this->num_agents);
    if (screen > 0) {
        // std::cout << "######################################" << std::endl;
        // printf("Num of agents: %d\n", this->num_agents);
        spdlog::info("Plan Result:");
    }

    for (int i = 0; i < this->num_agents; i++) {
        if (screen > 0) {
            std::cout << "Agent " << i << ": ";
        }
        bool task_finished = false;
        // The first simulation_window steps of the path are collision free
        for (int t = 0; t < this->simulation_window; t++) {
            // Infer task id
            State s = this->solution[i][t];
            int task_id = -1;
            Task curr_goal = this->goal_locations[i];
            // We can only finish a task at the end of the simulation window.
            if (curr_goal.loc == s.location && !task_finished) {
                task_id = curr_goal.id;
                task_finished = true;  // Task is finished
            }
            new_mapf_plan[i].emplace_back(this->G->getRowCoordinate(s.location),
                                          this->G->getColCoordinate(s.location),
                                          static_cast<double>(t), task_id);
            if (screen > 0) {
                std::cout << "(" << this->G->getRowCoordinate(s.location) << ","
                          << this->G->getColCoordinate(s.location) << ","
                          << s.orientation << "," << task_id << ")->";
                if (task_id >= 0) {
                    std::cout << "End of task " << task_id << " at step " << t
                              << "->";
                }
            }
        }
        if (screen > 0) {
            std::cout << std::endl;
        }
    }
    return new_mapf_plan;
}
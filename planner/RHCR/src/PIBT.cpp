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
    this->curr_occupied.resize(this->G.size(), -1);
    this->next_occupied.resize(this->G.size(), -1);

    if (solution.size() == 0)
        solution.resize(num_of_agents);

    // Clear previous solutions, if any
    for (int i = 0; i < num_of_agents; i++) {
        solution[i].clear();
        solution[i].resize(this->simulation_window);
        // Initialize the solution with the start state
        solution[i][0] = starts[i];
        this->curr_occupied[starts[i].location] = i;
    }

    // `goals_mem` is a memory of goals for each agent. Reached goals will be
    // removed.
    this->goals_mem = goal_locations;

    // Order of agents
    vector<int> agents(num_of_agents);
    for (int i = 0; i < num_of_agents; i++) {
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
            // If one of the agents has no goals, do not compare
            if (this->goals_mem[a].empty() || this->goals_mem[b].empty())
                return false;
            double dist_a = this->G.heuristics.at(
                this->goals_mem[a][0].location)[solution[a][t - 1].location];
            double dist_b = this->G.heuristics.at(
                this->goals_mem[b][0].location)[solution[b][t - 1].location];
            // Prefer the agent with smaller distance to its goal
            if (dist_a != dist_b)
                return dist_a < dist_b;
            // If the distances are equal, break ties randomly
            return false;
        };
        std::shuffle(agents.begin(), agents.end(), this->gen);
        std::sort(agents.begin(), agents.end(), agents_cmp);
        // TODO: Use fancier priority order
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
            // Update the current and next occupied locations
            this->next_occupied[solution[k][t].location] = -1;
            if (this->curr_occupied[solution[k][t - 1].location] == k) {
                // The agent is still occupying the location
                this->curr_occupied[solution[k][t - 1].location] = -1;
            }
            // Update the current occupied location
            this->curr_occupied[solution[k][t].location] = k;

            // Check for any goals reached
            State state = solution[k][t];
            if (!this->goals_mem[k].empty() &&
                state.location == this->goals_mem[k].begin()->location) {
                this->goals_mem[k].erase(this->goals_mem[k].begin());
            }
        }

        // Print the content of goal_mem:
        if (screen > 1) {
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
    list<State> next_states_ = this->G.get_neighbors(start_state);
    vector<State> next_states(next_states_.begin(), next_states_.end());

    // Sort the next states by action cost + heuristic values, breaking ties
    // randomly.
    auto action_cmp = [&](const State &n1, const State &n2) {
        double cost1 = this->G.get_weight(start_state.location, n1.location);
        double cost2 = this->G.get_weight(start_state.location, n2.location);
        double h1 = this->G.heuristics.at(goal.location)[n1.location];
        double h2 = this->G.heuristics.at(goal.location)[n2.location];
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

    // Shuffle the next states to break ties randomly, then sort
    std::shuffle(next_states.begin(), next_states.end(), this->gen);
    std::sort(next_states.begin(), next_states.end(), action_cmp);

    for (auto next_state : next_states) {
        // avoid vertex conflicts
        if (this->next_occupied[next_state.location] != -1)
            continue;
        // avoid edge conflicts
        if (a_j != -1 && next_state.location == solution[a_j][from_t].location)
            continue;

        // reserve the next state
        this->next_occupied[next_state.location] = a_i;
        this->solution[a_i][from_t + 1] = next_state;

        // Another agent a_k is currently occupying the next state. Replan it,
        // it is not planned.
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
    this->solution[a_i][from_t + 1] = start_state.wait();
    return false;
}

void PIBT::print_results() const {
    std::cout << "PIBT*:Succeed," << runtime << "," << std::endl;
}

void PIBT::save_results(const std::string &fileName,
                        const std::string &instanceName) const {
    std::ofstream stats;
    stats.open(fileName, std::ios::app);
    stats << runtime << "," << num_restarts << "," << num_restarts << ","
          << num_expanded << "," << num_generated << "," << solution_cost << ","
          << min_sum_of_costs << "," << avg_path_length << "," << "0" << ","
          << instanceName << std::endl;
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
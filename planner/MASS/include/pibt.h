#pragma once
#include <ctime>

#include "common.h"
#include "graph.h"
#include "instance.h"
#include "states.h"
#include "task.h"

class PIBT {
public:
    double runtime = 0;              // total runtime of the algorithm
    bool solution_found = false;     // whether a solution is found
    vector<vector<State>> solution;  // solution paths for each agent

    // Runs the algorithm until the problem is solved or time is exhausted
    bool run(const vector<State> &starts,
             const vector<vector<Task>> &goal_locations);

    string get_name() const {
        return "PIBT";
    }

    void save_results(const std::string &fileName,
                      const std::string &instanceName) const;
    void save_search_tree(const std::string &fileName) const {
    }
    void save_constraints_in_goal_node(const std::string &fileName) const {
    }
    void clear();

    // a_i: the agent to be planned
    // a_j: the agent where a_i inherits priority from, -1 if None
    // Return true if a valid path is found, false otherwise.s
    bool pibt_funct(int a_i, int a_j, State start_state, Task goal_locations,
                    int from_t);

    // The path planner is never used in PIBT, it is kept for compatibility
    PIBT(shared_ptr<Graph> G, int simulation_window = 100, int screen = 0,
         int seed = 0, int num_agents = 0)
        : G(G),
          simulation_window(simulation_window),
          screen(screen),
          num_agents(num_agents) {
        this->gen = mt19937(seed);
    };
    ~PIBT() {
    }

    vector<vector<tuple<int, int, double, int>>> getPaths();

private:
    void print_results() const;
    shared_ptr<Graph> G;  // the graph of the environment
    int simulation_window;
    int screen;
    int num_agents = 0;  // number of agents

    // loc -> agent id, where the agents are currently and nextly occupied
    vector<int> curr_occupied;
    vector<int> next_occupied;
    mt19937 gen;
    vector<vector<Task>> goal_locations;
    vector<bool> is_waiting;  // is the agent waiting at the current timestep
    vector<vector<Task>> goals_mem;  // Memory of goals for each agent
};

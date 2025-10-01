#pragma once
#include <ctime>

#include "backup_planners/MAPFSolver.h"
#include "backup_planners/SIPP.h"

class PIBT : public MAPFSolver {
public:
    uint64_t num_expanded;
    uint64_t num_generated;
    uint64_t num_restarts;

    // Runs the algorithm until the problem is solved or time is exhausted
    bool run(const vector<State> &starts,
             const vector<vector<Task>> &goal_locations, int time_limit,
             const vector<int> &waited_time = vector<int>()) override;

    string get_name() const override {
        return "PIBT";
    }

    void save_results(const std::string &fileName,
                      const std::string &instanceName) const override;
    void save_search_tree(const std::string &fileName) const override {
    }
    void save_constraints_in_goal_node(
        const std::string &fileName) const override {
    }
    void clear() override;

    // a_i: the agent to be planned
    // a_j: the agent where a_i inherits priority from, -1 if None
    // Return true if a valid path is found, false otherwise.s
    bool pibt_funct(int a_i, int a_j, State start_state, Task goal_locations,
                    int from_t);

    // The path planner is never used in PIBT, it is kept for compatibility
    PIBT(const BasicGraph &G, SingleAgentSolver &path_planner);
    ~PIBT() {
    }

private:
    void print_results() const;
    vector<bool> is_waiting;  // is the agent waiting at the current timestep
    vector<vector<Task>> goals_mem;  // Memory of goals for each agent

    // loc -> agent id, where the agents are currently and nextly occupied
    vector<int> curr_occupied;
    vector<int> next_occupied;
};

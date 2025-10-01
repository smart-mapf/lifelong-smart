#pragma once
#include "backup_planners/BasicGraph.h"
#include "backup_planners/ReservationTable.h"
#include "backup_planners/Task.h"

class SingleAgentSolver {
public:
    bool prioritize_start;
    double suboptimal_bound;
    bool hold_endpoints;
    bool consider_task_wait = false;

    uint64_t num_expanded;
    uint64_t num_generated;
    double runtime;

    // int map_size;
    double path_cost;
    double min_f_val;  // min f-val seen so far
    // number of conflicts between this agent to all the other agents
    int num_of_conf;

    int rotation_time = 1;  // number of timesteps to rotate. should be >= 1

    unordered_map<int, double> travel_times;

    double compute_h_value(const BasicGraph& G, State curr, int goal_id,
                           const vector<Task>& goal_location) const;

    virtual Path run(const BasicGraph& G, const State& start,
                     const vector<Task>& goal_location, ReservationTable& RT,
                     const int agent_waited_time = 0) = 0;
    virtual string getName() const = 0;
    SingleAgentSolver()
        : suboptimal_bound(1),
          num_expanded(0),
          num_generated(0),
          min_f_val(0),
          num_of_conf(0) {
    }

protected:
    double focal_bound;
};

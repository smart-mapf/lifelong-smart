#pragma once
#include <rpc/client.h>
#include <rpc/rpc_error.h>

#include <nlohmann/json.hpp>

#include "BasicSystem.h"
#include "SMARTGraph.h"

using json = nlohmann::json;

class SMARTSystem : public BasicSystem {
public:
    SMARTSystem(SMARTGrid& G, MAPFSolver& solver);
    ~SMARTSystem();

    json simulate(int simulation_time);
    json summarizeResult();
    json summarizeCurrResult(int summarize_interval);
    json warmup(int warmup_time);
    json update_gg_and_step(int update_gg_interval);
    int total_sim_time;
    void set_total_sim_time(int total_sim_time, int warmup_time);
    // void update_task_dist();

private:
    SMARTGrid& G;
    unordered_set<int> held_endpoints;
    std::vector<string> next_goal_type;
    int task_id = 0;  // A global task ID for all tasks in the system.

    void initialize();
    void initialize_start_locations();
    void initialize_goal_locations();
    void update_goal_locations();
    int gen_next_goal(int agent_id, bool repeat_last_goal = false);
    int sample_workstation();
    // int sample_end_points();
    void update_start_locations(
        vector<pair<double, double>>& start_locs,
        set<int> finished_tasks_id);
    tuple<vector<double>, double, double> edge_pair_usage_mean_std(
        vector<vector<double>>& edge_usage);
    tuple<vector<vector<vector<double>>>, vector<vector<double>>>
    convert_edge_usage(vector<vector<double>>& edge_usage);

    // Used for workstation sampling
    discrete_distribution<int> workstation_dist;
    discrete_distribution<int> end_points_dist;
    mt19937 gen;

    // Used for SMART
    vector<vector<tuple<int, int, double, int>>> convert_path_to_smart();
};

#pragma once
#include <nlohmann/json.hpp>

#include "BasicSystem.h"
#include "GreyOrangeGraph.h"
#include "common.h"

using json = nlohmann::json;

class GreyOrangeSystem : public BasicSystem {
public:
    GreyOrangeSystem(GreyOrangeGrid& G, MAPFSolver& solver,
                     string task_file = "", string agent_file = "");
    ~GreyOrangeSystem();

    json simulate(int simulation_time);
    json summarizeResult();
    json summarizeCurrResult(int summarize_interval);
    json warmup(int warmup_time);
    json update_gg_and_step(int update_gg_interval);
    int total_sim_time;
    void set_total_sim_time(int total_sim_time, int warmup_time);

private:
    GreyOrangeGrid& G;
    unordered_set<int> held_endpoints;
    std::vector<string> next_goal_type;

    void initialize();
    void initialize_start_locations();
    void initialize_goal_locations();
    void update_goal_locations(int t);
    vector<Task> gen_next_goal(int agent_id);
    int sample_workstation();
    // int sample_end_points();
    void update_start_locations(int t);
    bool congested() const;
    tuple<vector<double>, double, double> edge_pair_usage_mean_std(
        vector<vector<double>>& edge_usage);
    tuple<vector<vector<vector<double>>>, vector<vector<double>>>
    convert_edge_usage(vector<vector<double>>& edge_usage);

    // Agent and task files
    string task_file;
    string agent_file;
    bool random_task = true;  // If true, tasks are randomly generated
    // <agent id, list<tuple<locations, wait_time> > >
    vector<list<Task>> tasks;
    bool load_agents();
    bool load_tasks();

    // Used for workstation sampling
    discrete_distribution<int> workstation_dist;
    discrete_distribution<int> end_points_dist;
    mt19937 gen;

    // Queue mechanism
    // Return true if the agent must go to a queue location
    bool update_queue_goal_locations(int k, int goal_loc,
                                     vector<Task>& new_goals);
};

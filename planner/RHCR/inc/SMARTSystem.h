#pragma once
#include <rpc/client.h>
#include <rpc/rpc_error.h>

#include <nlohmann/json.hpp>

#include "BasicSystem.h"
#include "SMARTGraph.h"

using json = nlohmann::json;

class SMARTSystem : public BasicSystem {
public:
    SMARTSystem(SMARTGrid& G, MAPFSolver& solver, string task_file = "");
    ~SMARTSystem();

    json simulate(int simulation_time);
    json summarizeResult();
    json summarizeCurrResult(int summarize_interval);
    json warmup(int warmup_time);
    json update_gg_and_step(int update_gg_interval);
    int total_sim_time;
    void set_total_sim_time(int total_sim_time, int warmup_time);
    // void update_task_dist();
    bool congested() const;
    void solve();

private:
    SMARTGrid& G;
    unordered_set<int> held_endpoints;
    std::vector<string> next_goal_type;
    int task_id = 0;  // A global task ID for all tasks in the system.
    string task_file;

    void initialize();
    void initialize_start_locations();
    void initialize_goal_locations();
    void update_goal_locations();
    int gen_next_goal(int agent_id, bool repeat_last_goal = false);
    int sample_workstation();
    // int sample_end_points();
    void update_start_locations(vector<tuple<double, double, int>>& start_locs,
                                set<int> finished_tasks_id);
    tuple<vector<double>, double, double> edge_pair_usage_mean_std(
        vector<vector<double>>& edge_usage);
    tuple<vector<vector<vector<double>>>, vector<vector<double>>>
    convert_edge_usage(vector<vector<double>>& edge_usage);
    string get_curr_stats() const;

    Path get_aisle_path(State start, const vector<Task>& tasks,
                        const set<int>& aisle, const int move[4]);

    // tasks
    bool random_task = true;  // If true, tasks are randomly generated
    // <agent id, list<Tasks > >
    vector<list<Task>> tasks;
    bool load_tasks();

    bool validateSolution() const;

    // Used for workstation sampling
    discrete_distribution<int> workstation_dist;
    discrete_distribution<int> end_points_dist;
    mt19937 gen;

    // Convert path to SMART format.
    vector<vector<tuple<int, int, double, int>>> convert_path_to_smart();

    // Aisle related
    // A rt used for aisle path planning.
    ReservationTable aisle_rt;
    // Initial aisle paths
    // agent_id -> Path. We need this because some agents might be exiting
    // the aisle, which has no task id at the moment.
    unordered_map<int, Path> init_aisle_paths;

    // Task id -> aisle path
    unordered_map<int, Path> aisle_paths;

    // Number of agents coming to each aisle. Aisle id -> number of agents.
    unordered_map<int, int> aisle_usage;
};

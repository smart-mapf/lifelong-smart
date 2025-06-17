#pragma once
#include "BasicSystem.h"
#include "SortingGraph.h"
#include <nlohmann/json.hpp>
#include "common.h"

using json = nlohmann::json;

class SortingSystem :
	public BasicSystem
{
public:
	SortingSystem(SortingGrid& G,
                  MAPFSolver& solver,
                  // from destination to chutes
                  std::map<int, vector<int>> chute_mapping,
                  std::string package_mode,
                  // list of packages, essentially list of destinations
                  vector<int> packages,
                  // distribution of packages on each destination
                  vector<double> package_dist_weight,
                  // task assignment cost
                  std::string task_assignment_cost,
                  vector<double> task_assignment_params);
	~SortingSystem();

	json simulate(int simulation_time);
	json summarizeResult();
	json summarizeCurrResult(int summarize_interval);
	json warmup(int warmup_time);
	json update_gg_and_step(int update_gg_interval);
	int total_sim_time;
	void set_total_sim_time(int total_sim_time, int warmup_time);
	// void update_task_dist();

private:
	SortingGrid& G;
	// unordered_set<int> held_endpoints;
	std::vector<string> next_goal_type;

	void initialize();
	void initialize_start_locations();
	void initialize_goal_locations();
	void update_goal_locations(int t);
	int gen_next_goal(int agent_id, int curr_loc);
    int sample_workstation();
	// int sample_end_points();
	void update_start_locations(int t);
    tuple<vector<double>, double, double> edge_pair_usage_mean_std(
        vector<vector<double>> &edge_usage);
    tuple<vector<vector<vector<double>>>, vector<vector<double>>> convert_edge_usage(vector<vector<double>> &edge_usage);

    // Used for workstation sampling
    // discrete_distribution<int> workstation_dist;
	// discrete_distribution<int> end_points_dist;
    mt19937 gen;

    // Package dist
    int package_id = 0;
    std::discrete_distribution<int> package_dist;
    std::string package_mode;
    // std::vector<int> prev_task_locs;
    std::vector<int> packages;
    std::vector<double> package_dist_weight;
    std::map<int, vector<int>> chute_mapping;

    // workstations and #robots that intends to go to this workstation
    boost::unordered_map<int, int> robots_in_workstations;
    // endpoints and #robots that intends to go to this endpoint
    boost::unordered_map<int, int> robots_in_endpoints;

    // task assign
    std::string task_assignment_cost;
    double assign_C = 8;
    std::vector<double> task_assignment_params;
    double compute_assignment_cost(
        int curr_loc, pair<int, int> workstation) const;
    int assign_workstation(int curr_loc) const;
    int assign_endpoint(int curr_loc, vector<int> endpoints) const;
    void check_n_agents_sum();
    vector<int> sample_package();
    void update_n_agents(Task task);

};


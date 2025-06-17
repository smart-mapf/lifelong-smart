#pragma once
#include "BasicSystem.h"
#include "ManufactureGraph.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class ManufactureSystem :
	public BasicSystem
{
public:
	ManufactureSystem(const ManufactureGrid& G, MAPFSolver& solver);
	~ManufactureSystem();

	json simulate(int simulation_time);


private:
	const ManufactureGrid& G;
	unordered_set<int> held_endpoints;

    // Current goal type of all agents
	vector<int> curr_goal_types;

	void initialize();
	void initialize_start_locations();
	void initialize_goal_locations();
	void update_goal_locations();
	int gen_next_goal(int agent_id, bool repeat_last_goal=false);
    bool congested() const;
    void update_start_locations();

    json warmup(int warmup_time) override;
	json update_gg_and_step(int update_gg_interval) override;
    void set_total_sim_time(int total_sim_time, int warmup_time) override;

    // Used for workstation sampling
    discrete_distribution<int> workstation_dist;
    mt19937 gen;
};


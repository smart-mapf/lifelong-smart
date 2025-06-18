#pragma once
#include "SIPP.h"
#include "MAPFSolver.h"
#include <ctime>

// WHCA* with random restart
class WHCAStar :
	public MAPFSolver
{
public:

    uint64_t num_expanded;
    uint64_t num_generated;
    uint64_t num_restarts;

	ReservationTable initial_rt;

    vector<Path> initial_solution;

    // Runs the algorithm until the problem is solved or time is exhausted
    bool run(const vector<State>& starts,
             const vector< vector<Task > >& goal_locations,
             int time_limit,
             const vector<int>& waited_time = vector<int>()) override;

	string get_name() const override {return "WHCA"; }

    void save_results(const std::string &fileName, const std::string &instanceName) const override;
	void save_search_tree(const std::string &fileName) const override {}
	void save_constraints_in_goal_node(const std::string &fileName) const override {}
	void clear() override;

    WHCAStar(const BasicGraph& G, SingleAgentSolver& path_planner);
    ~WHCAStar() {}


private:

    void print_results() const;
};

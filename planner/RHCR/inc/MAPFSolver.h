#pragma once
#include "PBSNode.h"
#include "SIPP.h"
#include "Task.h"
#include <ctime>

// Base class for MAPF solvers
class MAPFSolver
{
public:
    int k_robust;
    int window;
	bool hold_endpoints;
    bool consider_task_wait = false;

	double runtime;
    int screen;
    int seed;
    mt19937 gen;

	bool solution_found;
	double solution_cost;
	double avg_path_length;
	double min_sum_of_costs;
    vector<Path> solution;

	// initial data
	ReservationTable initial_rt;
	vector<Path> initial_paths;
    list< tuple<int, int, int> > initial_constraints; // <agent, location, timestep>:
    // only this agent can stay in this location before this timestep.
	list<const Path*> initial_soft_path_constraints; // the paths that all agents try to avoid
	unordered_map<int, double> travel_times;


	SingleAgentSolver& path_planner;
	// Runs the algorithm until the problem is solved or time is exhausted
    virtual bool run(
        const vector<State>& starts,
        // an ordered list of tuple of <location, release time, wait time>
        const vector< vector<Task > >& goal_locations,
        int time_limit,
        // The number of timesteps each agent has waited. Used if considering
        // tasking wait time
        const vector<int>& waited_time = vector<int>()) = 0;


	MAPFSolver(const BasicGraph& G, SingleAgentSolver& path_planner);
	~MAPFSolver();

	// Save results
	virtual void save_results(const std::string &fileName, const std::string &instanceName) const = 0;
	virtual void save_search_tree(const std::string &fileName) const = 0;
	virtual void save_constraints_in_goal_node(const std::string &fileName) const = 0;
	virtual void clear() = 0;

	virtual string get_name() const = 0;

    const BasicGraph& G;
    vector<State> starts;
    vector< vector<Task > > goal_locations;
    int num_of_agents;
	int time_limit;
    int n_iter_limit;

    // validate
    bool validate_solution();

    void print_solution() const;

protected:
    vector<vector<bool> > cat; // conflict avoidance table
    vector<unordered_set< pair<int, int> > > constraint_table;
    ReservationTable rt;
};


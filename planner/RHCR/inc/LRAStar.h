#pragma once
#include "MAPFSolver.h"


class LRAStar: public MAPFSolver
{
public:
	int simulation_window;

    uint64_t num_wait_commands;
    uint64_t num_expanded;
    uint64_t num_generated;

    // Runs the algorithm until the problem is solved or time is exhausted.
    // Current implementation can only deal with k_robust <= 1.
    void resolve_conflicts(const vector<Path>& paths);
    // TODO: implement for larger k_robust


    void save_results(const std::string &fileName, const std::string &instanceName) const override;
	void save_search_tree(const std::string &fileName) const override {}
	void save_constraints_in_goal_node(
        const std::string &fileName) const override {}

    LRAStar(const BasicGraph& G, SingleAgentSolver& path_planner);

    bool run(const vector<State>& starts,
             const vector< vector<Task > >& goal_locations,
             int time_limit,
             const vector<int>& waited_time = vector<int>()) override;

	string get_name() const override {return "LRA"; }
	void clear() override {}

private:
    StateTimeAStar astar; // TODO: delete this
    unordered_map<int, int> curr_locations; // key = location, value = agent_id
    unordered_map<int, int> next_locations; // key = location, value = agent_id
    // vector<list<pair<int, int> > > trajectories;


    void print_results() const;
    void wait_command(int agent, int timestep,
                      vector<list<pair<int, int> >::const_iterator >& traj_pointers);
    void wait_command(int agent, int timestep, vector<int>& path_pointers);

	Path find_shortest_path(
        const State& start,
        const vector<Task >& goal_location);
};


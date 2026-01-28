#pragma once
#include "backup_planners/FailPolicy.h"

class LRAStar : public FailPolicy {
public:
    uint64_t num_wait_commands;
    uint64_t num_expanded;
    uint64_t num_generated;

    // Runs the algorithm until the problem is solved or time is exhausted.
    // Current implementation can only deal with k_robust <= 1.
    void resolve_conflicts(const vector<Path>& paths);
    // TODO: implement for larger k_robust

    void save_results(const std::string& fileName,
                      const std::string& instanceName) const override;
    void save_search_tree(const std::string& fileName) const override {
    }
    void save_constraints_in_goal_node(
        const std::string& fileName) const override {
    }

    LRAStar(const BasicGraph& G, SingleAgentSolver& path_planner,
            shared_ptr<HeuristicTableBase> heuristic_table,
            const boost::program_options::variables_map vm);

    bool run(const vector<State>& starts,
             const vector<vector<Task> >& goal_locations,
             const vector<Path>& guide_paths = vector<Path>(),
             int time_limit = 60,
             const vector<int>& waited_time = vector<int>()) override;

    string get_name() const override {
        return "LRA";
    }
    void clear() override {
    }

private:
    unordered_map<int, int> curr_locations;  // key = location, value = agent_id
    unordered_map<int, int> next_locations;  // key = location, value = agent_id
    // vector<list<pair<int, int> > > trajectories;

    void print_results() const;

    // void wait_command(
    //     int agent, int timestep,
    //     vector<list<pair<int, int> >::const_iterator>& traj_pointers);

    void wait_command(int agent, int timestep, vector<int>& path_pointers);

    Path find_shortest_path(const State& start,
                            const vector<Task>& goal_location);
};

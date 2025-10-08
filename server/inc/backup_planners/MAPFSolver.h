#pragma once
#include <ctime>

#include "backup_planners/SIPP.h"
#include "heuristics/BasicHeuristicTable.h"
#include "utils/States.h"
#include "utils/Task.h"

// Base class for MAPF solvers
class MAPFSolver {
public:
    int k_robust;
    int window;             // planning window
    int simulation_window;  // simulation window
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
    // <agent, location, timestep>:
    list<tuple<int, int, int>> initial_constraints;
    // only this agent can stay in this location before this timestep.
    // the paths that all agents try to avoid
    list<const Path*> initial_soft_path_constraints;
    // unordered_map<int, double> travel_times;

    SingleAgentSolver& path_planner;
    shared_ptr<HeuristicTableBase> heuristic_table;
    // Runs the algorithm until the problem is solved or time is exhausted
    virtual bool run(
        const vector<State>& starts,
        // an ordered list of tuple of <location, release time, wait time>
        const vector<vector<Task>>& goal_locations, int time_limit = 60,
        // The number of timesteps each agent has waited. Used if considering
        // tasking wait time
        const vector<int>& waited_time = vector<int>()) = 0;

    MAPFSolver(const BasicGraph& G, SingleAgentSolver& path_planner,
               shared_ptr<HeuristicTableBase> heuristic_table,
               const boost::program_options::variables_map vm);
    ~MAPFSolver();

    // Save results
    virtual void save_results(const std::string& fileName,
                              const std::string& instanceName) const = 0;
    virtual void save_search_tree(const std::string& fileName) const = 0;
    virtual void save_constraints_in_goal_node(
        const std::string& fileName) const = 0;
    virtual void clear() = 0;

    virtual string get_name() const = 0;

    const BasicGraph& G;
    // vector<State> starts;
    // vector<vector<Task>> goal_locations;
    int num_of_agents;
    int time_limit;
    int n_iter_limit;

    // validate
    bool validate_solution();

    void print_solution() const;

    bool is_initialized() const {
        return this->_is_initialized;
    }

    void set_initialized(bool initialized) {
        this->_is_initialized = initialized;
    }

    vector<vector<tuple<int, int, double, int>>> convert_path_to_smart(
        const vector<vector<Task>>& goal_locations);
    // void print_mapf_instance(vector<State> starts_,
    //                          vector<vector<Task>> goals_) const;
    bool congested() const;

protected:
    vector<vector<bool>> cat;  // conflict avoidance table
    vector<unordered_set<pair<int, int>>> constraint_table;
    ReservationTable rt;
    // Flag to indicate whether the solver has been initialized
    bool _is_initialized = false;
};

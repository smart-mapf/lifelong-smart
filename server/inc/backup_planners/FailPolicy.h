#pragma once
#include <ctime>

#include "backup_planners/SIPP.h"
#include "heuristics/BasicHeuristicTable.h"
#include "utils/States.h"
#include "utils/Task.h"



/**
 * @class FailPolicy
 * @brief The base class for all fail policies in LSMART.
 * @details This class defines the interface and common functionality for fail policies used in the LSMART framework. Fail policies are responsible for handling situations where the main MAPF solver fails to find a solution within the given constraints. To implement a new fail policy, inherit from this class and implement the pure virtual methods.
 */

class FailPolicy {
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


    /**
     * @brief Constructor for the FailPolicy class
     * @param G Reference to the BasicGraph representing the map
     * @param path_planner Reference to the SingleAgentSolver used for single agent path planning
     * @param heuristic_table Shared pointer to the HeuristicTableBase for heuristic calculations
     * @param vm Boost program options variable map for configuration
     */
    FailPolicy(const BasicGraph& G, SingleAgentSolver& path_planner,
               shared_ptr<HeuristicTableBase> heuristic_table,
               const boost::program_options::variables_map vm);
    ~FailPolicy();

    /**
     * @brief Given a MAPF instance, solve it using the fail policy
     * @param starts Vector of start states for each agent
     * @param goal_locations Vector of goal locations for each agent
     * @param guide_paths Optional vector of guide paths for each agent. If the
     * MAPF solver fails, the guide paths are usually the partial solution
     * returned by the MAPF solver.
     * @param time_limit Time limit (in seconds) for the fail policy to find a
     * solution
     * @param waited_time Vector of timesteps each agent has already waited.
     * Used if considering tasking wait time.
     * @return True if a solution is found, false otherwise
     */
    virtual bool run(const vector<State>& starts,
                     const vector<vector<Task>>& goal_locations,
                     const vector<Path>& guide_paths = vector<Path>(),
                     int time_limit = 60,
                     const vector<int>& waited_time = vector<int>()) = 0;


    // Save results
    /**
     * @brief Save the results of the fail policy to a file.
     * @details This function can be left unimplemented if not needed.
     * @param fileName Name of the file to save the results to
     * @param instanceName Name of the MAPF instance
     */
    virtual void save_results(const std::string& fileName,
                              const std::string& instanceName) const = 0;

    /**
     * @brief Clear any internal data structures used by the fail policy.
     */
    virtual void clear() = 0;

    /**
     * @brief Get the name of the fail policy.
     * @return Name of the fail policy as a string.
     */
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

    /**
     * @brief Convert the solution of the fail policy from the internal
     * representation (``vector<Path>``) to SMART path format
     * (``vector<vector<tuple<int, int, double, int>>>``).
     * @param goal_locations The goal locations for each agent
     * @return The converted SMART path format
     */
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

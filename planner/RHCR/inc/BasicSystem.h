#pragma once
#include "BasicGraph.h"
#include "States.h"
#include "PriorityGraph.h"
#include "PBS.h"
#include "WHCAStar.h"
#include "ECBS.h"
#include "LRAStar.h"
#include "Task.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;


class BasicSystem
{
public:
    // params for MAPF algotithms
	MAPFSolver& solver;
	bool hold_endpoints;
	bool useDummyPaths;
    int time_limit;
    int overall_time_limit;
    clock_t start_time;
    int travel_time_window;
	//string potential_function;
	//double potential_threshold;
	//double suboptimal_bound;
    int screen;
	bool log;
    int num_of_drives;
    int seed;
    int simulation_window;
    int planning_window;
    int simulation_time;
    bool save_result;
    bool save_solver;
    bool stop_at_traffic_jam;

    // params for drive model
    bool consider_rotation;
    int k_robust;

    BasicSystem(const BasicGraph& G, MAPFSolver& solver);
    ~BasicSystem();

	// TODO
    /*bool load_config(std::string fname);
    bool generate_random_MAPF_instance();
    bool run();
	void print_MAPF_instance() const;
	void save_MAPF_instance(std::string fname) const;
	bool read_MAPF_instance(std::string fname);*/

    // I/O
    std::string outfile;
    void save_results();
	double saving_time = 0; // time for saving results to files, in seconds
    int num_of_tasks; // number of finished tasks

	list<int> new_agents; // used for replanning a subgroup of agents

    // used for MAPF instance
    vector<State> starts;

    // Each goal contains (location, hold_time, wait_time), where
    // 1. location is the index of the goal in the map
    // 2. hold_time is used by hold_endpoint and dummy_path
    // 3. wait_time is the number of timesteps the agent has to wait at the
    //    goal. Used if considering waiting time at the goal
    // 4. orientation is the expected orientation of the robot at the goal
    //    locations
    vector<vector<Task>> goal_locations;
	// unordered_set<int> held_endpoints;

    int timestep;

    // Number of timesteps the agents has waited
    // Used if considering waiting time at the goal
    vector<int> waited_time;

    // Bool to remember whether each agent is doing task.
    // Used if considering waiting time at the goal
    vector<bool> is_tasking;


    // Rotational time
    int rotation_time = 1;
    // Number of timesteps the agents has waited in order to finish rotation
    // Used if considering rotation and the rotation time is > 1
    vector<int> rotate_time;

    // Bool to remember whether each agent is rotating.
    // Used if considering rotation and the rotation time is > 1
    vector<bool> is_rotating;

    // record movements of drives
    std::vector<Path> paths;
    std::vector<std::list<Task> > finished_tasks; // location + finish time

    int task_dist_update_interval = -1;
	std::string task_dist_type="default";


    bool congested() const;
	bool check_collisions(const vector<Path>& input_paths) const;

    // update
    void update_start_locations();
    void update_travel_times(unordered_map<int, double>& travel_times);
    void update_paths(const std::vector<Path*>& MAPF_paths, int max_timestep);
    void update_paths(const std::vector<Path>& MAPF_paths, int max_timestep);
    void update_initial_paths(vector<Path>& initial_paths) const;
    void update_initial_constraints(list< tuple<int, int, int> >& initial_constraints) const;
    
	void add_partial_priorities(const vector<Path>& initial_paths, PriorityGraph& initial_priorities) const;
	list<Task> move(); // return finished tasks
	void solve();
	void initialize_solvers();
	bool load_records();
	bool load_locations();

    // Online GGO
    virtual json warmup(int warmup_time) = 0;
	virtual json update_gg_and_step(int update_gg_interval) = 0;
    virtual void set_total_sim_time(int total_sim_time, int warmup_time) = 0;

    // Queue mechanism, used by GreyOrangeSystem
    bool queue_mechanism = false;

    // loc -> which agent is going to the location
    unordered_map<int, int> goal_occupied;

    // List of agents that are originally in the queue but are aborted from the
    // queue because the workstation they are trying to go to is now free.
    // This is naturally sorted by the distance to the workstation because the
    // given list of queue locations for each workstation is assumed to be
    // sorted by their distance to the workstation.
    vector<int> queue_aborted_agt;

protected:
	bool solve_by_WHCA(vector<Path>& planned_paths,
		const vector<State>& new_starts, const vector< vector<Task > >& new_goal_locations);
    bool LRA_called = false;

    // Total number of MAPF algo calls
    int n_mapf_calls = 0;

    // When MAPF solver runs out of time, we use a rule-based planner (such as
    // LRA*) to plan paths in the current window.
    int n_rule_based_calls = 0;

private:
	const BasicGraph& G;
};


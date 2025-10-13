#pragma once

#include <rpc/server.h>

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

#include "ADG.h"
#include "backup_planners/GuidedPIBT.h"
#include "backup_planners/LRAStar.h"
#include "backup_planners/PIBT.h"
#include "heuristics/BasicHeuristicTable.h"
#include "heuristics/LandmarkHeuristicTable.h"
#include "heuristics/LazyHeuristicTable.h"
#include "task_assigners/BasicTaskAssigner.h"
#include "task_assigners/DistinctOneGoalTaskAssigner.h"
#include "task_assigners/OneGoalTaskAssigner.h"
#include "utils/SMARTGraph.h"
#include "utils/common.h"

class ExecutionManager {
public:
    ExecutionManager(const boost::program_options::variables_map vm);

    // Compute and save stats
    void saveStats();

    // Get the current simulation step (tick) as the min of all robots' ticks
    int getCurrSimStep();

    // Return true if the planner has never been invoked
    bool plannerNeverInvoked();

    // Return true if the simulation has finished
    bool simulationFinished();

    // Return true if at least `ticks` simulation ticks have elapsed since last
    // planner invocation
    bool simTickElapsedFromLastInvoke(int ticks);

    // Update the simulation step tick
    void updateSimStep(string RobotID);

    // Return true if we should stop the simulation, either due to congestion or
    // due to the simulation finished
    bool stopSimulation();

    // Setup the backup planner
    void setupBackupPlanner();

    // Setup task assigner
    void setupTaskAssigner();

    // Freeze the simulation if necessary
    void freezeSimulationIfNecessary(string RobotID);

    // Obtain current location of the robots
    string getRobotsLocation();

    // Add a new MAPF plan to the ADG. Use backup planner if necessary
    // new_plan: a vector of paths, each path is a vector of (row, col, t,
    // task_id) Raw plan --> points --> Steps --> Actions
    void addNewPlan(string& new_plan_json_str);

    // A robot finished an action, update ADG
    string actionFinished(string& robot_id_str, int node_ID);

    // Initialize the ExecutionManager with the initial location of a robot
    void init(string RobotID, tuple<int, int> init_loc);

    // Obtain new batch of actions from ADG for a robot
    SIM_PLAN obtainActionsFromADG(string RobotID);

    // Return true if we should invoke the planner
    bool invokePlanner();

    // Record stats per tick
    void recordStatsPerTick();

    // Return true if the simulation is currently frozen
    bool isSimulationFrozen() const {
        return this->freeze_simulation;
    }

    // Return true if the ADG has been initialized
    bool isADGInitialized() const {
        return this->adg->initialized;
    }

    int getRPCPort() const {
        return this->port;
    }

private:
    // Setup the heuristic table
    void setupHeuristicTable();

    // Setup single agent path planner
    void setupSingleAgentPlanner();

    // Convert paths from planner to spatial guide paths
    vector<Path> convertPlanToGuidePaths(const vector<vector<UserState>>& plan);

    std::shared_ptr<ADG> adg;
    bool flipped_coord = true;
    int screen = 0;
    int port;             // RPC port number
    int seed;             // Random seed for the simulation
    int numRobots = 0;    // Number of robots
    PlanParser parser;    // Parser to convert mapf plan to actions
    int look_ahead_tick;  // Look ahead # in number of simulation ticks

    // Status of the simulation
    vector<int> tick_per_robot;      // Number of ticks each robot has moved
    bool freeze_simulation = false;  // Whether to freeze the simulation
    bool congested_sim = false;      // Whether the simulation is congested
    // The last tick when the planner was invoked
    int prev_invoke_planner_tick = -1;
    bool planner_running = false;  // Whether the planner is currently running
    int total_sim_step_tick;       // Total number of simulation ticks
    int ticks_per_second;          // Ticks per second for the simulation
    vector<robotState> curr_robot_states;  // Current states of all robots
    // (row, col, orientation) or (col, row, orientation)
    // vector<tuple<double, double, int>> robots_location;
    // vector of <n_finished tasks, time in sim seconds>
    vector<tuple<int, double>> tasks_finished_per_sec;

    // Planner invoke policy.
    // 1. default: invoke planner every sim_window_tick
    // 2. no_action: invoke planner when the number of actions left for a robot
    //    is less than look_ahead_dist, and at least sim_window_tick has passed
    //    since last invocation.
    string planner_invoke_policy = "default";
    int sim_window_tick = 50;  // Invoke planner every sim_window_tick

    // Stats related
    // Start time of the simulation
    string output_filename;
    std::chrono::steady_clock::time_point start_time;
    double overall_runtime = 0.0;      // Overall runtime of the simulation
    vector<int> planner_invoke_ticks;  // Sim ticks when planner is invoked
    string planner_stats = "{}";       // Store planner stats in JSON format

    // Backup planner related
    SMARTGrid G;
    shared_ptr<SingleAgentSolver> path_planner = nullptr;
    shared_ptr<MAPFSolver> backup_planner = nullptr;
    shared_ptr<HeuristicTableBase> heuristic_table = nullptr;
    bool save_stats = false;
    boost::program_options::variables_map _vm;

    // Task assigner related
    shared_ptr<BasicTaskAssigner> task_assigner = nullptr;
    string task_assigner_type;
};
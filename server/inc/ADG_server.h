#pragma once

#include <rpc/server.h>

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

#include "ADG.h"
#include "common.h"

#ifdef DEBUG
#define DEBUG_AGENT 13
#else
#define DEBUG_AGENT
#endif

std::mutex globalMutex;

class ADG_Server {
public:
    ADG_Server(boost::program_options::variables_map vm);
    void saveStats();
    int getCurrSimStep();

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
    vector<tuple<double, double, int>> robots_location;
    // vector of <n_finished tasks, time in sim seconds>
    vector<tuple<int, double>> tasks_finished_per_sec;

    // Planner invoke policy.
    // 1. default: invoke planner every sim_window_tick
    // 2. rhcre: invoke planner when the number of actions left for a robot
    //    is less than look_ahead_dist, and at least sim_window_tick has passed
    //    since last invocation.
    string planner_invoke_policy = "default";
    int sim_window_tick = 50;  // Invoke planner every sim_window_tick

    // Stats related
    // Start time of the simulation
    string output_filename;
    chrono::steady_clock::time_point start_time;
    double overall_runtime = 0.0;  // Overall runtime of the simulation
    string planner_stats = "{}";   // Store planner stats in JSON format

private:
    bool save_stats = false;
};
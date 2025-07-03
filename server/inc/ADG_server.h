#pragma once

#include <rpc/server.h>

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>     // For file operations
#include <functional>  // For std::bind
#include <iostream>
#include <map>  // For mapping robot actions
#include <mutex>
#include <numeric>  // For std::accumulate
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "ADG.h"
#include "json.hpp"

using json = nlohmann::json;

#ifdef DEBUG
#define DEBUG_AGENT 13
#else
#define DEBUG_AGENT
#endif

std::mutex globalMutex;

class ADG_Server {
public:
    ADG_Server(int num_robots, std::string target_output_filename,
               bool save_stats, int screen, int port, int total_sim_step_tick,
               int ticks_per_second, int look_ahead_dist, int sim_window_tick,
               int seed);
    void saveStats();
    int getCurrSimStep();

    // TODO@jingtian: move some of them into private
    std::shared_ptr<ADG> adg;
    // std::shared_ptr<MobileTaskManager> mobile_manager;
    // std::shared_ptr<PickTaskManager> picker_manager;
    bool flipped_coord = true;

    int screen = 0;
    int port;
    bool freeze_simulation = false;
    bool congested_sim = false;
    int prev_invoke_planner_tick = -1;
    bool planner_running = false;
    int seed;

    // Remember a tick count for each robot as the "simulation clock time".
    int total_sim_step_tick;

    int ticks_per_second;  // Ticks per second for the simulation

    int sim_window_tick = 50;  // Invoke planner every sim_window_tick

    std::vector<std::deque<std::shared_ptr<MobileRobotTask>>> curr_mobile_tasks;
    std::vector<robotState> curr_robot_states;

    // stats data
    // std::vector<double> agent_finish_time;
    // std::vector<int> agent_finish_sim_step;
    // std::vector<bool> agents_finish;
    std::string output_filename;
    int numRobots = 0;
    // double latest_arr_sim_step = 0;
    std::vector<std::pair<double, double>> robots_location;
    // std::vector<MobileAction> current_robots_goal_type;

    // double total_wait_time = 0;
    // int transport_capacity = 0;

    // bool debug_set_flag = false;
    vector<int> tick_per_robot;
    // int time_step_tick = 0;  // Current simulation step tick
    // int total_confirmed_picks = 0;
    // std::vector<int> confirmed_picks_by_genre;
    // std::vector<int> genre_finish_steps;

    // double min_nonzero() {
    //     int min_val = std::numeric_limits<int>::max();
    //     for (int v : genre_finish_steps) {
    //         if (v != 0.0 && v < min_val) {
    //             min_val = v;
    //         }
    //     }
    //     return min_val == std::numeric_limits<int>::max() ? 0.0 : min_val; //
    //     or throw if all are 0
    // }

    PlanParser parser;
    clock_t start_time;
    double overall_runtime = 0.0;  // Overall runtime of the simulation

private:
    //    int type1EdgeCount = 0;
    //    int type2EdgeCount = 0;
    //    int moveActionCount = 0;
    //    int rotateActionCount = 0;
    //    int consecutiveMoveSequences = 0;
    //    int totalNodes = 0;
    //    std::set<std::pair<int, int>> conflict_pairs;
    // std::string path_filename_;
    bool save_stats = false;
    double raw_plan_cost = -1.0;
};
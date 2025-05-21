#pragma once


#include <rpc/server.h>
#include <string>
#include <iostream>
#include <vector>
#include <tuple>
#include <stdexcept>
#include <mutex>
#include <sstream>
#include <functional>   // For std::bind
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>    // For file operations
#include <map>        // For mapping robot actions
#include <numeric> // For std::accumulate

#include "ADG.h"
#include "json.hpp"
#include "mobile_task.h"
#include "picker_task.h"

using json = nlohmann::json;


#ifdef DEBUG
#define DEBUG_AGENT 13
#else
#define DEBUG_AGENT
#endif

std::mutex globalMutex;


class ADG_Server{
public:
    ADG_Server(int num_robots, int num_pickers, std::string& target_output_filename, std::string map_name, std::string scen_name, std::string method_name);
    void saveStats();
    
    // TODO@jingtian: move some of them into private
    std::shared_ptr<ADG> adg;
    std::shared_ptr<MobileTaskManager> mobile_manager;
    std::shared_ptr<PickTaskManager> picker_manager;
    bool flipped_coord = true;

    std::vector<std::deque<std::shared_ptr<MobileRobotTask>>> curr_mobile_tasks;
    std::vector<robotState> curr_robot_states;

    // stats data
    std::vector<double> agent_finish_time;
    std::vector<int> agent_finish_sim_step;
    std::vector<bool> agents_finish;
    std::string output_filename;
    std::string curr_map_name;
    std::string curr_scen_name;
    std::string curr_method_name;
    int numRobots = 0;
    int step_cnt = 0;
    double latest_arr_sim_step = 0;
    std::vector<std::pair<double, double>> robots_location;
    std::vector<MobileAction> current_robots_goal_type;


    bool debug_set_flag = false;
    int total_confirmed_picks = 0;
    std::vector<int> confirmed_picks_by_genre;
    std::vector<int> genre_finish_steps;

private:
//    int type1EdgeCount = 0;
//    int type2EdgeCount = 0;
//    int moveActionCount = 0;
//    int rotateActionCount = 0;
//    int consecutiveMoveSequences = 0;
//    int totalNodes = 0;
//    std::set<std::pair<int, int>> conflict_pairs;
    // std::string path_filename_;
    double raw_plan_cost = -1.0;
};
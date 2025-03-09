#include <rpc/client.h>
#include <string>
#include <iostream>
#include "ADG.h"
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
#include "json.hpp"
using json = nlohmann::json;

// #define DEBUG

#ifdef DEBUG
#define DEBUG_AGENT 13
#else
#define DEBUG_AGENT
#endif

std::mutex globalMutex;


class ADG_Server{
public:
    ADG_Server(int num_robots, std::string& target_output_filename, std::string map_name, std::string scen_name, std::string method_name);
    void saveStats();
    
    // TODO@jingtian: move some of them into private
    std::shared_ptr<ADG> adg;
    bool flipped_coord = true;

    // std::vector<int> flags;
    // std::vector<int> adg_queue;
    std::map<int, std::string> robotIDTOStartIndex;
    std::map<std::string, int> startIndexToRobotID;
    std::vector<std::vector<Action>> plans;
    std::vector<std::vector<int>> outgoingEdgesByRobot;
    std::vector<double> agent_finish_time;
    std::vector<int> agent_finish_sim_step;
    int latest_arr_sim_step = 0;
    bool all_agents_finished = false;
    std::vector<bool> agents_finish;
    std::string output_filename;
    std::string curr_map_name;
    std::string curr_scen_name;
    std::string curr_method_name;
    int numRobots = 0;
    int step_cnt = 0;

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
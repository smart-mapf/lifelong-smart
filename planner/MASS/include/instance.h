#pragma once

#include <algorithm>  // std::shuffle
#include <boost/tokenizer.hpp>
#include <chrono>
#include <random>  // std::default_random_engine
#include <vector>

#include "common.h"
#include "graph.h"
#include "task.h"

using namespace std;

struct Agent {
    int id = -1;
    int start_location = -1;
    orient start_o = orient::East;
    // int goal_location = -1;
    // orient end_o = orient::None;
    vector<Task> goal_locations;

    // These default values are set to match the default values in RobotMotion.
    // double v_min = DEFAULT_V_MIN;
    // double v_max = DEFAULT_V_MAX;
    // double a_min = DEFAULT_A_MAX * -1.0;  // negative acceleration
    // double a_max = DEFAULT_A_MAX;         // positive acceleration
    // double rotation_cost = DEFAULT_ROTATE_COST;
    // double turn_back_cost = DEFAULT_TURN_BACK_COST;
    shared_ptr<RobotMotion> bot_motion;
    double length = DEFAULT_LENGTH;  // radius of the robot

    /*For time window*/
    bool is_solved = false;
    double earliest_start_time = 0.0;
    double init_velocity = 0.0;
    Path previous_path;
    std::vector<int> trajectory;

    Agent() = default;

    Agent(int start_loc, orient start_ori, vector<Task> goal_locations,
          std::shared_ptr<RobotMotion> bot_motion)
        : start_location(start_loc),
          start_o(start_ori),
          goal_locations(goal_locations),
          bot_motion(bot_motion)
    //   v_min(bot_motion->V_MIN),
    //   v_max(bot_motion->V_MAX),
    //   a_min(-bot_motion->A_MAX),
    //   a_max(bot_motion->A_MAX),
    //   rotation_cost(bot_motion->ROTATE_COST),
    //   turn_back_cost(bot_motion->TURN_BACK_COST)
    {
    }

    /*statistic for result*/
    double total_cost = 0;
    double tmp_cost = 0;
};

// Currently only works for undirected unweighted 4-neighbor grids
class Instance {
public:
    std::vector<Agent> agents;
    int num_of_agents = 0;
    int use_sps_type = 0;  // 0 for Binary, 1 for Bezier
    bool use_pe = false;
    double simulation_window = 10;  // Simulation (planning) window in seconds

    // graph representation of the map
    shared_ptr<Graph> graph;

    Instance() = default;
    Instance(shared_ptr<Graph> graph, shared_ptr<RobotMotion> bot_motion,
             json mapf_instance, bool use_partial_expansion = false,
             int used_sps_solver = 0, int screen = 0,
             double simulation_window = 10);
    ~Instance();

    void GetRawReservationTable();
    bool loadAgents(json mapf_instance);

    int getDefaultNumberOfAgents() const {
        return num_of_agents;
    }
    void GetAgents(std::vector<Agent>& agents_list);

    void GetReservationTable(ReservationTable& rt) {
        rt = raw_rv_tbl;
    };

    void saveAgents(string filename) const;

private:
    ReservationTable raw_rv_tbl;
    shared_ptr<RobotMotion> bot_motion;

private:
    // initialize new [rows x cols] map with random obstacles
    void generateRandomAgents();
    int screen;
};

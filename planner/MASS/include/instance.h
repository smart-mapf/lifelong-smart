#pragma once

#include <algorithm>  // std::shuffle
#include <boost/tokenizer.hpp>
#include <chrono>
#include <random>  // std::default_random_engine
#include <vector>

#include "common.h"
#include "graph.h"
#include "task.h"
#include "task_assigner.h"
#include "states.h"

using namespace std;

// struct position_t
// {
//     float x;
//     float y;
//     position_t()=default;
//     position_t(float a, float b) {
//         x = a;
//         y = b;
//     };
// };

// struct Vertex
// {
//     std::string name;
//     position_t pos;
//     std::set<int> generalizedVertexConflicts;
//     int id;
// };

// struct Edge
// {
//     std::string name;
//     float length;
// };

// typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
// Vertex, Edge> searchGraph_t;

struct Agent {
    int id = -1;
    int start_location = -1;
    orient start_o = orient::East;
    // int goal_location = -1;
    // orient end_o = orient::None;
    vector<Task> goal_locations;

    double v_min = V_MIN;
    double v_max = V_MAX;
    double a_min = -0.5;
    double a_max = 0.5;
    double length = 0.4;  // radius of the robot
    double rotation_cost = ROTATE_COST;
    double turn_back_cost = TURN_BACK_COST;

    /*For time window*/
    bool is_solved = false;
    double earliest_start_time = 0.0;
    double init_velocity = 0.0;
    Path previous_path;
    std::vector<int> trajectory;

    Agent() = default;

    Agent(int start_loc, orient start_ori, vector<Task> goal_locations)
        : start_location(start_loc),
          start_o(start_ori),
          goal_locations(goal_locations) {
    }

    /*statistic for result*/
    double total_cost = 0;
    double tmp_cost = 0;
};

// Currently only works for undirected unweighted 4-neighbor grids
class Instance {
public:
    // int num_of_cols = 0;
    // int num_of_rows = 0;
    // int map_size = 0;
    std::vector<Agent> agents;
    int num_of_agents = 0;
    int use_sps_type = 0;  // 0 for Binary, 1 for Bezier
    bool use_pe = false;
    double simulation_window = 10;  // Simulation (planning) window in seconds

    // graph representation of the map
    shared_ptr<Graph> graph;
    shared_ptr<TaskAssigner> task_assigner;

    Instance() = default;
    Instance(shared_ptr<Graph> graph, shared_ptr<TaskAssigner> task_assigner,
             vector<tuple<double, double, int>>& start_locs,
             set<int> finished_tasks_id, bool use_partial_expansion = false,
             int used_sps_solver = 0, int screen = 0,
             double simulation_window = 10);

    // void printAgents() const;

    void GetRawReservationTable();

    bool loadAgents(vector<tuple<double, double, int>>& start_locs,
                    set<int> finished_tasks_id);
    // inline bool isObstacle(int loc) const { return my_map[loc]; }
    // inline bool validMove(int curr, int next) const
    // {
    // 	if (next < 0 || next >= map_size)
    // 		return false;
    // 	if (my_map[next])
    // 		return false;
    // 	return getManhattanDistance(curr, next) < 2;
    // }

    // void finishAgent(int goal)
    // {
    // 	my_map[goal] = 1;
    // }

    // list<int> getNeighbors(int curr) const;
    // void getNeighbors(int curr, orient curr_o, std::vector<std::tuple<int,
    // orient, double>>& neighbors) const; void getNeighbors(int curr, orient
    // direct, Neighbors& neighbor) const; void getInverseNeighbors(int curr,
    // orient direct, Neighbors& neighbor) const;

    // inline int linearizeCoordinate(int row, int col) const { return (
    // this->num_of_cols * row + col); } inline int getRowCoordinate(int id)
    // const { return id / this->num_of_cols; } inline int getColCoordinate(int
    // id) const { return id % this->num_of_cols; } inline pair<int, int>
    // getCoordinate(int id) const { return make_pair(id / this->num_of_cols, id
    // % this->num_of_cols); } inline int getCols() const { return num_of_cols;
    // } inline int getRows() const { return num_of_rows; }

    // inline int getManhattanDistance(int loc1, int loc2) const
    // {
    // 	int loc1_x = getRowCoordinate(loc1);
    // 	int loc1_y = getColCoordinate(loc1);
    // 	int loc2_x = getRowCoordinate(loc2);
    // 	int loc2_y = getColCoordinate(loc2);
    // 	return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
    // }

    // static inline int getManhattanDistance(const pair<int, int>& loc1, const
    // pair<int, int>& loc2)
    // {
    // 	return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
    // }

    // int walkCounterClockwise(int from, int to) const
    // {
    // 	assert(validMove(from, to));
    // 	int dir = turnLeft(to - from);
    // 	while (!validMove(from, from + dir))
    // 		dir = turnLeft(dir);
    // 	return from + dir;
    // }
    // inline int turnLeft(int dir) const
    // {
    // 	if (dir ==  1)
    // 		return -num_of_cols;
    // 	else if (dir == -num_of_cols)
    // 		return - 1;
    // 	else if (dir == -1)
    // 		return num_of_cols;
    // 	else if (dir == num_of_cols)
    // 		return 1;
    // 	else
    // 		return 0;
    //  }
    // inline int turnRight(int dir) const
    // {
    // 	if (dir == 1)
    // 		return num_of_cols;
    // 	else if (dir == num_of_cols)
    // 		return -1;
    // 	else if (dir == -1)
    // 		return -num_of_cols;
    // 	else if (dir == -num_of_cols)
    // 		return 1;
    // 	else
    // 		return 0;
    // }
    // int getDegree(int loc) const // return the number of neighbors
    // {
    // 	assert(loc >= 0 && loc < map_size && !my_map[loc]);
    // 	int degree = 0;
    // 	if (0 <= loc - num_of_cols && !my_map[loc - num_of_cols])
    // 		degree++;
    // 	if (loc + num_of_cols < map_size && !my_map[loc + num_of_cols])
    // 		degree++;
    // 	if (loc % num_of_cols > 0 && !my_map[loc - 1])
    // 		degree++;
    // 	if (loc % num_of_cols < num_of_cols - 1 && !my_map[loc + 1])
    // 		degree++;
    // 	return degree;
    // }

    int getDefaultNumberOfAgents() const {
        return num_of_agents;
    }
    void GetAgents(std::vector<Agent>& agents_list);

    vector<State> getStartLocations() const {
        vector<State> starts;
        for (const auto& agent : agents) {
            starts.push_back(State(agent.start_location, 0, agent.start_o));
        }
        return starts;
    };

    // int GetNumOfVertices() {return map_size;};
    void GetReservationTable(ReservationTable& rt) {
        rt = raw_rv_tbl;
    };

private:
    // int moves_offset[MOVE_COUNT];
    // vector<bool> my_map;
    // string map_fname;
    // string agent_fname;
    // string agent_indices;
    ReservationTable raw_rv_tbl;

    //   vector<int> start_locations;
    //   vector<int> goal_locations;

private:
    // bool loadMap();
    // void printMap() const;
    // void saveMap() const;

    // void saveAgents() const;

    // void generateConnectedRandomGrid(int rows, int cols, int obstacles); //
    // initialize new [rows x cols] map with random obstacles
    void generateRandomAgents();
    int screen;
    // bool addObstacle(int
    // obstacle); // add this obsatcle only if the map is still connected

    // Class  SingleAgentSolver can access private members of Node
    // friend class SingleAgentSolver;
};

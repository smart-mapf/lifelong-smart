#pragma once
#include "Graph.h"
#include "common.h"

struct Task {
    int id;
    int loc;

    Task(int id, int loc) : id(id), loc(loc) {
    }
};

// Currently only works for undirected unweighted 4-nighbor grids
class Instance {
public:
    // int num_of_cols;
    // int num_of_rows;
    // int map_size;
    int screen = 0;
    Graph graph;  // graph representation of the map
    int simulation_window = 0;  // simulation window for the planner

    // enum valid_moves_t { NORTH, EAST, SOUTH, WEST, WAIT_MOVE, MOVE_COUNT };
    // // MOVE_COUNT is the enum's size

    Instance() {
    }
    Instance(const Graph& graph, vector<Task> goal_locations, int screen,
             int task_id, int simulation_window);

    void setGoalLocations(const vector<Task>& goal_locations) {
        this->goal_locations = goal_locations;
    }

    vector<Task> getGoalTasks() const {
        return goal_locations;
    }

    vector<int> getGoalLocations() const {
        vector<int> goal_locs;
        for (const auto& task : goal_locations) {
            goal_locs.push_back(task.loc);
        }
        return goal_locs;
    }

    int getTaskId() const {
        return task_id;
    }

    void printAgents() const;

    // inline bool isObstacle(int loc) const {
    //     return my_map[loc];
    // }
    // inline bool validMove(int curr, int next) const;
    // list<int> getNeighbors(int curr) const;

    // inline int linearizeCoordinate(int row, int col) const {
    //     return (this->num_of_cols * row + col);
    // }
    // inline int getRowCoordinate(int id) const {
    //     return id / this->num_of_cols;
    // }
    // inline int getColCoordinate(int id) const {
    //     return id % this->num_of_cols;
    // }
    // inline pair<int, int> getCoordinate(int id) const {
    //     return make_pair(id / this->num_of_cols, id % this->num_of_cols);
    // }
    // inline int getCols() const {
    //     return num_of_cols;
    // }

    // inline int getManhattanDistance(int loc1, int loc2) const {
    //     int loc1_x = getRowCoordinate(loc1);
    //     int loc1_y = getColCoordinate(loc1);
    //     int loc2_x = getRowCoordinate(loc2);
    //     int loc2_y = getColCoordinate(loc2);
    //     return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
    // }

    // inline int getManhattanDistance(const pair<int, int>& loc1,
    //                                 const pair<int, int>& loc2) const {
    //     return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
    // }

    // int getDegree(int loc) const {
    //     assert(loc >= 0 && loc < map_size && !my_map[loc]);
    //     int degree = 0;
    //     if (0 <= loc - num_of_cols && !my_map[loc - num_of_cols])
    //         degree++;
    //     if (loc + num_of_cols < map_size && !my_map[loc + num_of_cols])
    //         degree++;
    //     if (loc % num_of_cols > 0 && !my_map[loc - 1])
    //         degree++;
    //     if (loc % num_of_cols < num_of_cols - 1 && !my_map[loc + 1])
    //         degree++;
    //     return degree;
    // }

    int getDefaultNumberOfAgents() const {
        return num_of_agents;
    }

    bool loadAgents(std::vector<std::pair<double, double>>& start_locs,
                    set<int> finished_tasks_id);
    // void printMap() const;
    // void saveInstance();

    // // nodes from and to are neighbors
    // double getWeight(int from, int to) const;

    // int getDirection(int from, int to) const;

private:
    // int moves_offset[MOVE_COUNT];
    // vector<bool> my_map;  // true if obstacle, false if free
    // string map_fname;
    // string agent_fname;
    // vector<vector<double>> weights;  // (directed) weighted 4-neighbor grid`

    int task_id = 0;
    int num_of_agents;
    vector<int> start_locations;
    vector<Task> goal_locations;
    // vector<int> free_locations;  // locations that are not obstacles

    // // Direction of movement
    // // 0: right, 1: up, 2: left, 3: down
    // int move[4];

    // bool loadMap();
    // bool loadMapFromBench();
    // bool loadMapFromJson();
    // void saveMap() const;
    // void update_map_weights(std::vector<double>& new_weights);
    //
    // bool loadAgents();
    // void saveAgents() const;

    int genGoal(set<int> to_avoid, int curr_goal, int agent_id);

    // void generateConnectedRandomGrid(
    //     int rows, int cols, int obstacles);  // initialize new [rows x cols]
    //     map
    //                                          // with random obstacles
    // // void generateRandomAgents(int warehouse_width);
    // bool addObstacle(
    //     int obstacle);  // add this obsatcle only if the map is still
    //     connected
    // bool isConnected(int start,
    //                  int goal);  // run BFS to find a path between start and
    //                              // goal, return true if a path exists.

    // int randomWalk(int loc, int steps) const;

    // Class  SingleAgentSolver can access private members of Node
    friend class SingleAgentSolver;
};

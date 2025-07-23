#pragma once
#include "common.h"
#include "task.h"

struct Neighbors {
    pair<int, orient> left;
    pair<int, orient> right;
    pair<int, orient> back;
    std::vector<int> forward_locs;
};

class Graph {
public:
    int num_of_cols;
    int num_of_rows;
    int map_size;
    int screen = 0;

    // heuristics for guidance graph, low level search optimize this.
    boost::unordered_map<int, vector<vector<double>>> heuristics;

    // distance heuristics, used for reasoning target conflicts
    // boost::unordered_map<int, vector<int>> d_heuristics;

    Graph() {
    }

    Graph(const string& map_fname, int screen);

    inline bool isObstacle(int loc) const {
        return my_map[loc];
    }
    // bool validMove(int curr, int next) const;
    inline bool validMove(int curr, int next) const {
        if (next < 0 || next >= map_size)
            return false;
        if (my_map[next])
            return false;
        return getManhattanDistance(curr, next) < 2;
    }
    list<int> getNeighbors(int curr) const;
    void getNeighbors(
        int curr, orient curr_o,
        std::vector<std::tuple<int, orient, double>>& neighbors) const;
    void getNeighbors(int curr, orient direct, Neighbors& neighbor) const;
    void getInverseNeighbors(int curr, orient direct,
                             Neighbors& neighbor) const;

    inline int linearizeCoordinate(int row, int col) const {
        return (this->num_of_cols * row + col);
    }
    inline int getRowCoordinate(int id) const {
        spdlog::debug("getRowCoordinate: id = {}", id);
        return id / this->num_of_cols;
    }
    inline int getColCoordinate(int id) const {
        return id % this->num_of_cols;
    }
    inline pair<int, int> getCoordinate(int id) const {
        return make_pair(id / this->num_of_cols, id % this->num_of_cols);
    }
    inline int getCols() const {
        return num_of_cols;
    }

    inline int getManhattanDistance(int loc1, int loc2) const {
        int loc1_x = getRowCoordinate(loc1);
        int loc1_y = getColCoordinate(loc1);
        int loc2_x = getRowCoordinate(loc2);
        int loc2_y = getColCoordinate(loc2);
        return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
    }

    inline int getManhattanDistance(const pair<int, int>& loc1,
                                    const pair<int, int>& loc2) const {
        return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
    }

    int getDegree(int loc) const {
        assert(loc >= 0 && loc < map_size && !my_map[loc]);
        int degree = 0;
        if (0 <= loc - num_of_cols && !my_map[loc - num_of_cols])
            degree++;
        if (loc + num_of_cols < map_size && !my_map[loc + num_of_cols])
            degree++;
        if (loc % num_of_cols > 0 && !my_map[loc - 1])
            degree++;
        if (loc % num_of_cols < num_of_cols - 1 && !my_map[loc + 1])
            degree++;
        return degree;
    }

    // void printMap() const;

    void computeHeuristics();
    vector<double> computeHeuristicsOneLoc(int root_location);
    std::vector<std::vector<double>> BackDijkstra(int root_location);

    // nodes from and to are neighbors
    double getWeight(int from, int to) const;

    int getDirection(int from, int to) const;

    int nEmptyLocations() const {
        return empty_locations.size();
    }

    int sampleEmptyLocation() const {
        if (empty_locations.empty()) {
            return -1;  // no empty locations
        }
        int idx = rand() % empty_locations.size();
        return empty_locations[idx];
    }

    int sampleWorkstation() const {
        if (workstations.empty()) {
            return -1;  // no workstations
        }
        int idx = rand() % workstations.size();
        return workstations[idx];
    }

    int sampleEndpoint() const {
        if (endpoints.empty()) {
            return -1;  // no endpoints
        }
        int idx = rand() % endpoints.size();
        return endpoints[idx];
    }

    int sampleWarehouseTaskLoc() const {
        if (warehouse_task_locs.empty()) {
            return -1;  // no warehouse task locations
        }
        int idx = rand() % warehouse_task_locs.size();
        return warehouse_task_locs[idx];
    }

    int sampleFreeLocation() const {
        if (free_locations.empty()) {
            return -1;  // no free locations
        }
        int idx = rand() % free_locations.size();
        return free_locations[idx];
    }

    int size() const {
        return map_size;
    }

    double getHeuristic(vector<Task> goals, int start_loc, orient start_ori,
                        int goal_id) const;

    int walkCounterClockwise(int from, int to) const {
        assert(validMove(from, to));
        int dir = turnLeft(to - from);
        while (!validMove(from, from + dir))
            dir = turnLeft(dir);
        return from + dir;
    }
    inline int turnLeft(int dir) const {
        if (dir == 1)
            return -num_of_cols;
        else if (dir == -num_of_cols)
            return -1;
        else if (dir == -1)
            return num_of_cols;
        else if (dir == num_of_cols)
            return 1;
        else
            return 0;
    }
    inline int turnRight(int dir) const {
        if (dir == 1)
            return num_of_cols;
        else if (dir == num_of_cols)
            return -1;
        else if (dir == -1)
            return -num_of_cols;
        else if (dir == -num_of_cols)
            return 1;
        else
            return 0;
    }

    int GetNumOfVertices() {
        return map_size;
    };

private:
    vector<bool> my_map;  // true if obstacle, false if free
    vector<string> types;
    string map_fname;
    string agent_fname;
    vector<vector<double>> weights;  // (directed) weighted 4-neighbor grid

    vector<int> free_locations;  // locations that are not obstacles
    vector<int> workstations;
    vector<int> endpoints;
    vector<int> warehouse_task_locs;  // workstations + endpoints
    vector<int>
        empty_locations;  // locations that are not obstacles or task_loc

    // Direction of movement
    // 0: right, 1: up, 2: left, 3: down
    int move[4];

    bool loadMap();
    bool loadMapFromBench();
    bool loadMapFromJson();
    void saveMap() const;
    void update_map_weights(std::vector<double>& new_weights);
    void setDefaultEdgeWeights();

    void printMap() const;

    // run BFS to find a path between start and goal, return true if a path
    // exists.
    // bool isConnected(int start, int goal) const;

    int randomWalk(int loc, int steps) const;

    // Classes that can access private members
    // friend class SingleAgentSolver;
    friend class Instance;
    friend class TaskAssigner;
};
#pragma once
#include "common.h"

class Graph {
public:
    int num_of_cols;
    int num_of_rows;
    int map_size;
    int screen = 0;

    // heuristics for guidance graph, low level search optimize this.
    unordered_map<int, vector<double>> heuristics;

    // distance heuristics, used for reasoning target conflicts
    unordered_map<int, vector<int>> d_heuristics;

    Graph() {
    }

    Graph(const string& map_fname, int screen);

    inline bool isObstacle(int loc) const {
        return my_map[loc];
    }
    bool validMove(int curr, int next) const;
    list<int> getNeighbors(int curr) const;

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

    tuple<vector<double>, vector<int>> computeHeuristicsOneLoc(
        int root_location);

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

private:
    vector<bool> my_map;  // true if obstacle, false if free
    vector<string> types;
    string map_fname;
    string agent_fname;
    vector<vector<double>> weights;  // (directed) weighted 4-neighbor grid

    vector<int> free_locations;  // locations that are not obstacles
    vector<int> workstations;
    vector<int> endpoints;
    vector<int> warehouse_task_locs; // workstations + endpoints
    vector<int> empty_locations; // locations that are not obstacles or task_loc

    // Direction of movement
    // 0: right, 1: up, 2: left, 3: down
    int move[4];

    bool loadMap();
    bool loadMapFromBench();
    bool loadMapFromJson();
    void saveMap() const;
    void update_map_weights(std::vector<double>& new_weights);
    void setDefaultEdgeWeights();

    // Classes that can access private members
    friend class SingleAgentSolver;
    friend class Instance;
    friend class TaskAssigner;
};
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

private:
    vector<bool> my_map;  // true if obstacle, false if free
    string map_fname;
    string agent_fname;
    vector<vector<double>> weights;  // (directed) weighted 4-neighbor grid

    vector<int> free_locations;  // locations that are not obstacles

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
};
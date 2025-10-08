#pragma once
#include "utils/States.h"
#include "utils/common.h"
#include "utils/enums.h"

class BasicGraph {
public:
    virtual bool load_map_from_jsonstr(string json_str, double left_w_weight,
                                       double right_w_weight) = 0;

    list<State> get_neighbors(const State& v) const;
    list<int> get_neighbors(int v) const;
    list<State> get_reverse_neighbors(const State& v) const;  // ignore time
    // fiducials from and to are neighbors
    double get_weight(int from, int to) const;
    vector<vector<double>> get_weights() const {
        return weights;
    }
    // return 0 if it is 0; return 1 if it is +-90; return 2 if it is 180
    int get_rotate_degree(int dir1, int dir2) const;

    void print_map() const;
    int get_rows() const {
        return rows;
    }
    int get_cols() const {
        return cols;
    }
    int size() const {
        return rows * cols;
    }

    bool valid_move(int loc, int dir) const {
        return (weights[loc][dir] < WEIGHT_MAX - 1);
    }
    int get_Manhattan_distance(int loc1, int loc2) const;
    void copy(const BasicGraph& copy);
    int get_direction(int from, int to) const;
    int get_direction_output(int from, int to) const;

    inline int getRowCoordinate(int id) const {
        return id / this->cols;
    }
    inline int getColCoordinate(int id) const {
        return id % this->cols;
    }
    inline int getCellId(int row_idx, int col_idx) const {
        return this->cols * row_idx + col_idx;
    }

    int move[4];
    vector<string> types;
    string map_name;
    int rows;
    int cols;
    vector<vector<double>> weights;  // (directed) weighted 4-neighbor grid
    bool consider_rotation;

    int screen;

    bool hold_endpoints;
    bool useDummyPaths;

    // Rotational time
    int rotation_time = 1;

    // Map from the orientation in SMART to the orientation in RHCR.
    const unordered_map<int, int> ORI_SMART_TO_RHCR = {
        {0, 1},
        {1, 0},
        {2, 3},
        {3, 2},
    };

    void set_grid_type(SMARTGridType type) {
        this->grid_type = type;
    }

    SMARTGridType get_grid_type() const {
        return this->grid_type;
    }

protected:
    // Type of the SMART grid.
    SMARTGridType grid_type = SMARTGridType::REGULAR;
};

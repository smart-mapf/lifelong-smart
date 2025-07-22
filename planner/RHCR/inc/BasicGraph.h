#pragma once
#include "common.h"
#include "States.h"
#include "enums.h"

#define WEIGHT_MAX INT_MAX/2


class BasicGraph
{
public:
    vector<std::string> types;

    string map_name;
	virtual bool load_map(string fname) = 0;
    virtual bool load_map_from_jsonstr(std::string json_str, double left_w_weight, double right_w_weight) = 0;
    virtual void preprocessing(bool consider_rotation, std::string log_dir) = 0;
    virtual void reset_weights(bool consider_rotation, std::string log_dir, bool optimize_wait, std::vector<double>& weights) = 0;

    list<State> get_neighbors(const State& v) const;
    list<int> get_neighbors(int v) const;
    list<State> get_reverse_neighbors(const State& v) const; // ignore time
    double get_weight(int from, int to) const; // fiducials from and to are neighbors
    vector<vector<double> > get_weights() const {return weights; }
    int get_rotate_degree(int dir1, int dir2) const; // return 0 if it is 0; return 1 if it is +-90; return 2 if it is 180

    void print_map() const;
    int get_rows() const { return rows; }
    int get_cols() const { return cols; }
    int size() const { return rows * cols; }

    bool valid_move(int loc, int dir) const {return (weights[loc][dir] < WEIGHT_MAX - 1); }
    int get_Manhattan_distance(int loc1, int loc2) const;
    int move[4];
    void copy(const BasicGraph& copy);
    int get_direction(int from, int to) const;
    int get_direction_output(int from, int to) const;

    inline int getRowCoordinate(int id) const { return id / this->cols; }
    inline int getColCoordinate(int id) const { return id % this->cols; }
    inline int getCellId(int row_idx, int col_idx)
    {
        return this->cols * row_idx + col_idx;
    }

	vector<double> compute_heuristics(int root_location); // compute distances from all lacations to the root location
    double get_heuristic(int goal_loc, int start_loc, int start_ori = -1) const;
	bool load_heuristics_table(std::ifstream& myfile);
	void save_heuristics_table(string fname);
    bool _save_heuristics_table;

    int rows;
    int cols;
    vector<vector<double> > weights; // (directed) weighted 4-neighbor grid
    bool consider_rotation;

    int screen;

    bool hold_endpoints;
	bool useDummyPaths;

    // Rotational time
    int rotation_time = 1;

    // Each workstation has a set of queue locations for robots to wait in a
    // queue. Used by the queueing mechanism of GreyOrange system.
    unordered_map<int, vector<int>> queue_locations;

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
    unordered_map<int, vector<double>> heuristics;

    // Type of the SMART grid.
    SMARTGridType grid_type = SMARTGridType::REGULAR;

};

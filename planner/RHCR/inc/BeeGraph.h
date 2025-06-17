#pragma once
#include "BasicGraph.h"


class BeeGraph :
	public BasicGraph
{
public:
	vector<int> flowers;
	vector<int> flower_demands;
	vector<int> flower_costs;
	vector<pair<int, int> > flower_time_windows;
	vector<int> initial_locations;
	int bee_capacity;
	int entrance; // vertex collisions at home will be ignored
	int num_of_bees;
	int max_timestep;
	int move_cost; // cost of a move action
	int wait_cost; // cost of a wait action
	bool load_map(string fname);
	bool load_Nathan_map(string fname);
	// void preprocessing(string fname, bool consider_rotation); // compute heuristics
	double loading_time = 0; // time for loading the map from files
	double preprocessing_time = 0; // in seconds

    bool load_map_from_jsonstr(std::string json_str, double left_w_weight, double right_w_weight);
    void preprocessing(bool consider_rotation, std::string log_dir);
    void reset_weights(bool consider_rotation, std::string log_dir, bool optimize_wait, std::vector<double>& weights);


};


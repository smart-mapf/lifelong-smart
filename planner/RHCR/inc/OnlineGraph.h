#pragma once
#include "BasicGraph.h"


class OnlineGrid :
	public BasicGraph
{
public:
	vector<int> entries;
	vector<int> exits;
    bool load_map(string fname);
    void preprocessing(bool consider_rotation, std::string log_dir); // compute heuristics
    bool load_map_from_jsonstr(std::string json_str, double left_w_weight, double right_w_weight);
    void reset_weights(bool consider_rotation, std::string log_dir, bool optimize_wait, std::vector<double>& weights);
};

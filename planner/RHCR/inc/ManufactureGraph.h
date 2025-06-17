#pragma once
#include "BasicGraph.h"
#include <nlohmann/json.hpp>
#include <random>
using json = nlohmann::json;

class ManufactureGrid : public BasicGraph
{
public:
    vector<int> agent_home_locations;

    // Types of manufacture stations
    int n_station_types;

    // Number of timesteps to wait at each type of stations.
    vector<int> station_wait_time;

    // 'n_station_types' types of "obstacles" (or manufacture stations). The
    // agents are required to traverse to an endpoint around each workstation
    // in order.
    // For example, suppose there are 3 types, at type 1,2,3, the robots will
    // wait for 1,5,10 timesteps, respectively, where 1,5,10 are configurable
    // numbers.
    // The agents cannot traverse through the manufacture stations.
    vector<vector<int>> manufacture_stations;

    // 'n_station_types' types of endpoints corresponding to the manufacture
    // stations.
    vector<vector<int>> endpoints;

    bool load_map(string fname);
    bool load_map_from_jsonstr(std::string json_str);

    // compute heuristics
    void preprocessing(bool consider_rotation, std::string log_dir);

    bool load_map_from_jsonstr(std::string json_str, double left_w_weight, double right_w_weight);
    void reset_weights(bool consider_rotation, std::string log_dir, bool optimize_wait, std::vector<double>& weights);


    double get_avg_task_len(
        unordered_map<int, vector<double>> heuristics) const;

private:
    bool load_weighted_map(string fname);
    bool load_unweighted_map(string fname);
    bool load_unweighted_map_from_json(json G_json);
    bool load_weighted_map_from_json(json G_json);
    void infer_endpoint_type();
    void print_map_info();
    void process_map_line(string line, int i);
    void set_unweighted_graph_weight();
};

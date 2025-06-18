#pragma once
#include <nlohmann/json.hpp>
#include <random>

#include "BasicGraph.h"
#include "TaskDistGenerator.h"

using json = nlohmann::json;

class GreyOrangeGrid : public BasicGraph {
public:
    vector<int> endpoints;
    vector<int> agent_home_locations;
    vector<int> workstations;
    vector<int> parking_locations;
    vector<double> workstation_weights;
    vector<double> end_points_weights;

    // Dummy function to get around inheritance issue
    bool load_map(string fname) {
        return false;
    }
    bool load_map(string fname, double left_w_weight,
                  double right_w_weight);
    bool load_map_from_jsonstr(std::string json_str);
    bool load_map_from_jsonstr(std::string json_str, double left_w_weight,
                               double right_w_weight) override;
    void preprocessing(bool consider_rotation,
                       std::string log_dir) override;  // compute heuristics
    void reset_weights(bool consider_rotation, std::string log_dir,
                       bool optimize_wait,
                       std::vector<double>& weights) override;
    void update_map_weights(std::vector<double>& new_weights);
    double get_avg_task_len(
        unordered_map<int, vector<double>> heuristics) const;
    int get_n_valid_edges() const;

    void initialize_end_points_weights() {
        this->end_points_weights.clear();
        this->end_points_weights.resize(this->endpoints.size(), 1.0);
    }
    void initialize_workstation_weights() {
        this->workstation_weights.clear();
        this->workstation_weights.resize(this->workstations.size(), 1.0);
    }
    void parseMap(std::vector<std::vector<double>>& map_e,
                  std::vector<std::vector<double>>& map_w);

private:
    // Number of valid edges.
    int n_valid_edges = -1;

    // Number of valid vertices
    int n_valid_vertices = -1;

    bool load_weighted_map(string fname);
    bool load_unweighted_map(string fname);
    bool load_unweighted_map_from_json(json G_json);
    bool load_weighted_map_from_json(json G_json);
};

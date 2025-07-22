#pragma once
#include "BasicGraph.h"
#include <nlohmann/json.hpp>
#include <random>

#include "TaskDistGenerator.h"

using json = nlohmann::json;

class SMARTGrid :
	public BasicGraph
{
public:
	vector<int> endpoints;
	vector<int> agent_home_locations;
    vector<int> workstations;
    vector<int> task_locations; // basically endpoints + workstations
    vector<double> workstation_weights;
    vector<double> end_points_weights;

    // Dummy function to get around inheritance issue
    bool load_map(string fname) { return false; }
    // bool load_map(string fname, double left_w_weight, double right_w_weight);
    bool load_map_from_jsonstr(
        std::string json_str, double left_w_weight, double right_w_weight) override;
    void preprocessing(bool consider_rotation, std::string log_dir) override; // compute heuristics
    void reset_weights(bool consider_rotation, std::string log_dir, bool optimize_wait, std::vector<double>& weights) override;
    void update_map_weights(std::vector<double>& new_weights);
    double get_avg_task_len(
        unordered_map<int, vector<double>> heuristics) const;
    int get_n_valid_edges() const;

    void initialize_end_points_weights(){
        this->end_points_weights.clear();
        this->end_points_weights.resize(this->endpoints.size(), 1.0);
    }
    void parseMap(std::vector<std::vector<double>>& map_e, std::vector<std::vector<double>>& map_w);
    void update_task_dist(std::mt19937& gen, std::string task_dist_type);

    bool in_aisle(int loc) const {
        return this->endpt_to_aisle.count(loc) > 0;
    }

    bool is_aisle_entry(int loc) const {
        return this->aisle_entries.count(loc) > 0;
    }

    set<int> get_aisle(int loc) const {
        if (this->in_aisle(loc)) {
            return this->aisles.at(this->endpt_to_aisle.at(loc));
        } else {
            throw std::runtime_error("SMARTGrid::get_aisle: location is not in an aisle.");
        }
    }

    int aisle_entry(int loc) const {
        if (this->in_aisle(loc)) {
            return this->endpt_to_aisle.at(loc);
        } else {
            throw std::runtime_error("SMARTGrid::aisle_entry: location is not in an aisle.");
        }
    }


private:
    // Number of valid edges.
    int n_valid_edges = -1;

    // Number of valid vertices
    int n_valid_vertices = -1;

    // Relevant for the SMARTGridType::ONE_BOT_PER_AISLE.
    // Map from node ID to aisle ID, where aisle ID is the id of the entry
    // point of the aisle.
    boost::unordered_map<int, int> endpt_to_aisle;
    boost::unordered_map<int, set<int>> aisles;
    set<int> aisle_entries;

    // bool load_weighted_map(string fname);
    // bool load_unweighted_map(
    //     string fname, double left_w_weight, double right_w_weight);
    bool load_unweighted_map_from_json(
        json G_json, double left_w_weight, double right_w_weight);
    bool load_weighted_map_from_json(
        json G_json, double left_w_weight, double right_w_weight);
    // Function to get the aisle information from the map. Applicable to the
    // SMARTGridType::ONE_BOT_PER_AISLE.
    void analyze_aisle();
};

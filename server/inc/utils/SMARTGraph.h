#pragma once
#include <random>

#include "utils/BasicGraph.h"

class SMARTGrid : public BasicGraph {
public:
    vector<int> endpoints;
    vector<int> agent_home_locations;
    vector<int> workstations;
    vector<int> task_locations;
    vector<int> free_locations;  // all non-obstacle locations
    vector<double> workstation_weights;
    vector<double> end_points_weights;

    bool load_map_from_jsonstr(std::string json_str, double left_w_weight,
                               double right_w_weight) override;
    void update_map_weights(std::vector<double> &new_weights);
    double get_avg_task_len(
        unordered_map<int, vector<double>> heuristics) const;
    int get_n_valid_edges() const;

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
            throw std::runtime_error(
                "SMARTGrid::get_aisle: location is not in an aisle.");
        }
    }

    int aisle_entry(int loc) const {
        if (this->in_aisle(loc)) {
            return this->endpt_to_aisle.at(loc);
        } else {
            throw std::runtime_error(
                "SMARTGrid::aisle_entry: location is not in an aisle.");
        }
    }

    set<int> get_aisle_entries() const {
        return this->aisle_entries;
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
    bool load_unweighted_map_from_json(json G_json, double left_w_weight,
                                       double right_w_weight);
    bool load_weighted_map_from_json(json G_json, double left_w_weight,
                                     double right_w_weight);
    // Function to get the aisle information from the map. Applicable to the
    // SMARTGridType::ONE_BOT_PER_AISLE.
    void analyze_aisle();
};

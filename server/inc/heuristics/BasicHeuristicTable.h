#pragma once
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>

#include "utils/BasicGraph.h"
#include "utils/common.h"
#include "heuristics/StateAStarNode.h"

class HeuristicTableBase {
public:
    HeuristicTableBase(const BasicGraph& G, vector<int> task_locations,
                       int seed, string log_dir = "",
                       bool save_heuristics_table = false)
        : G(G),
          task_locations(task_locations),
          _save_heuristics_table(save_heuristics_table),
          rng(seed) {
        this->save_path = boost::filesystem::path(log_dir);
        if (this->G.consider_rotation)
            this->save_path /=
                this->G.map_name + "_rotation_heuristics_table.txt";
        else
            this->save_path /= this->G.map_name + "_heuristics_table.txt";
    };
    ~HeuristicTableBase() {
        heuristics.clear();
    }
    virtual bool load_heuristics_table(std::ifstream& myfile);
    virtual void save_heuristics_table(string fname);
    virtual double get(int goal_location, int start_location) = 0;
    virtual void reset_heuristics() = 0;

protected:
    unordered_map<int, vector<double>> heuristics;
    const BasicGraph& G;
    vector<int> task_locations;
    boost::filesystem::path save_path;
    bool _save_heuristics_table = false;
    std::mt19937 rng;
};

// Basic heuristic table. Stores a map from goal location to a vector of
// heuristic values for all start locations.
class BasicHeuristicTable : public HeuristicTableBase {
public:
    BasicHeuristicTable(const BasicGraph& G, vector<int> task_locations,
                        int seed, string log_dir = "",
                        bool save_heuristics_table = false);
    ~BasicHeuristicTable() = default;
    // bool load_heuristics_table(std::ifstream& myfile) override;
    // void save_heuristics_table(string fname) override;
    double get(int goal_location, int start_location) override;
    void reset_heuristics() override;

protected:
    // compute distances from all lacations to the root location
    vector<double> compute_heuristics(int root_location);
};
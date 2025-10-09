#pragma once
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>

#include "heuristics/StateAStarNode.h"
#include "utils/BasicGraph.h"
#include "utils/common.h"

struct GuidePathHVal {
    // From current location to the closest point on the guide path
    double dp;
    // From the closest point on the guide path to the goal along the guide path
    double dg;
};

// Compare operation of GuidePathHVal
inline bool operator<(const GuidePathHVal& h1, const GuidePathHVal& h2) {
    // Prefer the one with smaller dp first
    if (h1.dp != h2.dp)
        return h1.dp < h2.dp;

    // If dp are the same, prefer the one with smaller dg
    return h1.dg < h2.dg;
}

inline std::ostream& operator<<(std::ostream& os, const GuidePathHVal& h) {
    os << "(dp: " << h.dp << ", dg: " << h.dg << ")";
    return os;
}

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
    virtual GuidePathHVal get_guide_path_h(int goal_location,
                                           int start_location,
                                           const Path& g_path) = 0;
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
    GuidePathHVal get_guide_path_h(int goal_location, int start_location,
                                   const Path& g_path) override;
    void reset_heuristics() override;

protected:
    // compute distances from all lacations to the root location
    vector<double> compute_heuristics(int root_location);
};
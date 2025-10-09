#pragma once
#include "heuristics/BasicHeuristicTable.h"

class LazyHeuristicTable : public HeuristicTableBase {
public:
    LazyHeuristicTable(const BasicGraph& G, vector<int> task_locations,
                       int seed, string log_dir = "",
                       bool save_heuristics_table = false);
    ~LazyHeuristicTable();
    double get(int goal_location, int start_location) override;
    GuidePathHVal get_guide_path_h(int goal_location, int start_location,
                                   const Path& g_path) override;
    void reset_heuristics() override;

protected:
    // Run lazy backward dijkstra rooted at `root_locations` and end at
    // `start_locations`
    double lazy_dijkstra(int root_location, int start_location);

    // Open list for each task location
    unordered_map<int, fibonacci_heap<StateAStarNode*,
                                      compare<StateAStarNode::compare_node>>>
        open_lists;
    unordered_map<int, unordered_set<StateAStarNode*, StateAStarNode::Hasher,
                                     StateAStarNode::EqNode>>
        nodes;
};
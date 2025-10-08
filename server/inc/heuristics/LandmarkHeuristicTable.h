#pragma once
#include "heuristics/BasicHeuristicTable.h"

class LandmarkHeuristicTable : public BasicHeuristicTable {
public:
    LandmarkHeuristicTable(const BasicGraph& G, vector<int> task_locations,
                           int seed, string log_dir = "",
                           bool save_heuristics_table = false,
                           int num_landmarks = 10,
                           string landmark_selection = "random");
    ~LandmarkHeuristicTable() = default;
    double get(int goal_location, int start_location) override;
    void reset_heuristics() override;

private:
    void select_landmarks(int num_landmarks, string landmark_selection);
    vector<int> landmarks;
};
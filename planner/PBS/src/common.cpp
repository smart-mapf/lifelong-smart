#include "common.h"

std::ostream& operator<<(std::ostream& os, const Path& path) {
    for (const auto& state : path) {
        os << state.location;  // << "(" << state.is_single() << "),";
    }
    return os;
}

bool isSamePath(const Path& p1, const Path& p2) {
    if (p1.size() != p2.size())
        return false;
    for (unsigned i = 0; i < p1.size(); i++) {
        if (p1[i].location != p2[i].location)
            return false;
    }
    return true;
}

bool congested(const vector<vector<tuple<int, int, double, int>>>& new_plan) {
    int wait_agents = 0;
    int n_agents = new_plan.size();
    for (const auto& path : new_plan) {
        int t = 0;
        int path_length = path.size();
        while (t < path_length &&
               std::get<0>(path[0]) == std::get<0>(path[t]) &&
               std::get<1>(path[0]) == std::get<1>(path[t])) {
            t++;
        }

        if (t == path_length)
            wait_agents++;
    }
    // more than half of robots didn't make progress
    // cout << "Number of waiting robots: " << wait_agents
    //      << ", Total number of robots: " << n_agents << endl;
    return wait_agents > n_agents / 2;
}

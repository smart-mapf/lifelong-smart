#include "common.h"

bool congested(
    const std::vector<std::vector<std::tuple<int, int, double, int>>>&
        new_plan) {
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

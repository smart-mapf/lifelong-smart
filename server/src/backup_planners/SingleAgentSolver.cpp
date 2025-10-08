#include "backup_planners/SingleAgentSolver.h"

double SingleAgentSolver::compute_h_value(
    const BasicGraph &G, shared_ptr<HeuristicTableBase> heuristic_table,
    State curr, int goal_id, const vector<Task> &goal_location) const {
    // std::cout << "begin"<<std::endl;
    int goal_loc = goal_location[goal_id].location;
    // double h = G.get_heuristic(goal_loc, curr.location, curr.orientation);
    double h = heuristic_table->get(goal_loc, curr.location);
    goal_id++;
    while (goal_id < (int)goal_location.size()) {
        // h += G.get_heuristic(goal_location[goal_id].location,
        //                      goal_location[goal_id - 1].location,
        //                      goal_location[goal_id - 1].orientation);
        h += heuristic_table->get(goal_location[goal_id].location,
                                  goal_location[goal_id - 1].location);
        goal_id++;
    }
    return h;
}

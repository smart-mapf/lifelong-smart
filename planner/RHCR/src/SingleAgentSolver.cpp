#include "SingleAgentSolver.h"

double SingleAgentSolver::compute_h_value(
    const BasicGraph &G, State curr, int goal_id,
    const vector<Task> &goal_location) const {
    // std::cout << "begin"<<std::endl;
    int goal_loc = goal_location[goal_id].location;
    // if (G.heuristics.find(goal_loc) == G.heuristics.end()){
    //     std::cout<< "error goal_loc ="<<goal_loc<<", not find in
    //     G.h"<<std::endl; exit(1);
    // }
    // if (G.get_heuristic(goal_loc).size() <= curr){
    //     std::cout << "error curr loc ="<<curr<<", larger than
    //     "<<G.get_heuristic(goal_loc).size()<<std::endl; exit(1);
    // }
    double h = G.get_heuristic(goal_loc, curr.location, curr.orientation);
    goal_id++;
    while (goal_id < (int)goal_location.size()) {
        // if (G.heuristics.find(goal_location[goal_id].location) ==
        // G.heuristics.end()){
        //     std::cout<< "error goal_loc
        //     ="<<goal_location[goal_id].location<<", not find in
        //     G.h"<<std::endl; exit(1);
        // }
        // if (G.get_heuristic(goal_location[goal_id].location).size() <=
        // goal_location[goal_id - 1].location){
        //     std::cout << "error curr loc ="<<curr<<", larger than
        //     "<<G.get_heuristic(goal_loc).size()<<std::endl; exit(1);
        // }
        h += G.get_heuristic(goal_location[goal_id].location,
                             goal_location[goal_id - 1].location,
                             goal_location[goal_id - 1].orientation);
        goal_id++;
    }
    // std::cout << "end, h = "<<h<<std::endl;
    return h;
}

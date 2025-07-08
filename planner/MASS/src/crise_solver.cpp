#include "crise_solver.h"

CRISE_Solver::CRISE_Solver(Agent& sipp_curr_agent,
                           std::deque<std::shared_ptr<IntervalEntry>>& sipp_result_nodes,
                           orient& curr_o): curr_agent(sipp_curr_agent), curr_o(curr_o)
{
    result_nodes.clear();
    for (auto& sipp_interval: sipp_result_nodes) {
        std::shared_ptr<IntervalEntry> tmp_interval = std::make_shared<IntervalEntry>();
        tmp_interval->step = sipp_interval->step;
        tmp_interval->t_min = sipp_interval->t_min;
        tmp_interval->t_max = sipp_interval->t_max;
        if ((tmp_interval->t_max - tmp_interval->t_min) < (2*curr_agent.length + CELL_DIS)/curr_agent.v_max) {
            interval_too_small = true;
        }
        // set it to nullptr, since we don't use this entry in tpg
        tmp_interval->prev_entry = nullptr;
        tmp_interval->location = sipp_interval->location;
        tmp_interval->interval_idx = sipp_interval->interval_idx;
        if (DEBUG_CRISE)
            printf("Forward interval at: %d, from %f to %f\n", tmp_interval->location,
                   tmp_interval->t_min, tmp_interval->t_max);
        result_nodes.push_back(tmp_interval);
    }
}

/**
 *
 * @param step The step of entries on the path
 * @return
 */
std::vector<std::pair<double, double>> CRISE_Solver::occupancyTimes(int length)
{
    std::vector<double> arr_time = arrTimes(length);
    std::vector<std::pair<double, double>> occupancy_interval;
    for (int i = 0; i < length; i++) {
        double t_lower = arr_time[std::max(0, i-1)];
        double t_upper = arr_time[std::min(length-1, i + 1)];
        occupancy_interval.emplace_back(t_lower, t_upper);
    }
    return occupancy_interval;
}


// Function to calculate the available start time ranges
bool CRISE_Solver::solve(shared_ptr<MotionNode> &solution) {
    solution = nullptr;
    int length = (int) result_nodes.size();
    std::vector<std::pair<double, double>> t_bound = occupancyTimes(length);
//    for (auto bound: t_bound){
//        printf("time bound: %f -> %f\n", bound.first, bound.second);
//    }
//    printf("\n");
    double wait_t = 0.0;
    double max_wait = INF;
    for (int i = 0; i < length; i++) {
        double t_lower = std::max(0.0, result_nodes[i]->t_min - t_bound[i].first);
        double t_upper = std::min(INF, result_nodes[i]->t_max - t_bound[i].second);
        if (t_upper < t_lower) {
            return false;
        }
        if (wait_t < t_lower) {
            wait_t = t_lower;
        }
        if (max_wait > t_upper) {
            max_wait = t_upper;
        }
    }
    if (max_wait < wait_t) {
        return false;
    }
//    printf("wait time is: %f\n", wait_t);
    solution = std::make_shared<MotionNode>();
    for (int i = 0; i < length; i++) {
//        double t_lower = t_bound[i].first + wait_t;
        double t_lower = 0;
        if (i == 0) {
            t_lower = result_nodes.front()->t_min;
        } else{
            t_lower = t_bound[i].first + wait_t;
        }
        double t_upper = std::min(INF, t_bound[i].second + wait_t);
        solution->local_path.emplace_back(result_nodes[i]->location, curr_o, t_lower, t_upper);
//        printf("At location: %d, t_lower is: %f, t_upper is: %f\n",
//               result_nodes[i]->location, t_lower, t_upper);
    }
    solution->optimal_T = optimal_T;
    return true;
}
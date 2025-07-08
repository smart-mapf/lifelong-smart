#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <ilcplex/ilocplex.h>

#include "tpg_node.h"
#include "berstein.h"
#include "common.h"
#include "instance.h"
#include "milp_cache.h"


class TPG_Solver{
public:
    TPG_Solver(Agent& sipp_curr_agent, std::deque<std::shared_ptr<IntervalEntry>>& sipp_result_nodes);
    bool solve(shared_ptr<MotionNode> &solution, shared_ptr<FailureCache> &failure_cache_ptr,
               shared_ptr<SuccessCache> &success_cache_ptr);
private:
    bool SolveBezier(double T_optimal, std::vector<double>& solution_control_points, Path& result_path);
    bool SolveSlackBezier(double T, std::vector<double>& solution_control_points,
                          Path& result_path, double& slack_var);
    bool RecurrentSolver(std::shared_ptr<MotionNode>& solution);
    bool SlackGradient(double target_t, double& gradient);
    void retrievePath(shared_ptr<MotionNode> &solution, shared_ptr<MotionNode> &bcp_solution);

private:
    Agent& curr_agent;
    IntervalSeq result_nodes;
    double min_init_t;
    bool interval_too_small = false;
    int n_points = CONTROL_POINTS_NUM;
    double total_runtime_ = 0.0;
};
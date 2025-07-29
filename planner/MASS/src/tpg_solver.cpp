#include "tpg_solver.h"

TPG_Solver::TPG_Solver(Agent& sipp_curr_agent, std::deque<std::shared_ptr<IntervalEntry>>& sipp_result_nodes): curr_agent(sipp_curr_agent)
{
    min_init_t = sipp_result_nodes[0]->t_min;
    result_nodes.clear();
    for (auto sipp_interval: sipp_result_nodes) {
        std::shared_ptr<IntervalEntry> tmp_interval = std::make_shared<IntervalEntry>();
        tmp_interval->step = sipp_interval->step;
        tmp_interval->t_min = sipp_interval->t_min - min_init_t;
        if (sipp_interval->t_max != INF) {
            tmp_interval->t_max = sipp_interval->t_max - min_init_t;
        } else {
            tmp_interval->t_max = INF;
        }
        if ((tmp_interval->t_max - tmp_interval->t_min) < (2*curr_agent.length + CELL_DIS)/curr_agent.bot_motion->V_MAX) {
            interval_too_small = true;
        }
        // set it to nullptr, since we don't use this entry in tpg
        tmp_interval->prev_entry = nullptr;
        tmp_interval->location = sipp_interval->location;
        tmp_interval->interval_idx = sipp_interval->interval_idx;
        result_nodes.push_back(tmp_interval);
    }
}

bool TPG_Solver::SolveBezier(
        double T_optimal,
        std::vector<double>& solution_control_points,
        Path& result_path)
{
    double a_min = curr_agent.bot_motion->A_MAX * -1.0;
    double a_max = curr_agent.bot_motion->A_MAX;
    double v_min = curr_agent.bot_motion->V_MIN;
    double v_max = curr_agent.bot_motion->V_MAX;
    double t_length = T_optimal;
    int traj_size = (int) result_nodes.size();

    BernsteinPolynomial bern_poly(n_points, t_length);
    // First order derivative of bernstein polynomial
    BernsteinPolynomial first_order_deriv = bern_poly.Derivative();
    // Second order derivative of bernstein polynomial
    BernsteinPolynomial second_order_deriv = first_order_deriv.Derivative();

    IloEnv env = IloEnv();
    IloModel model = IloModel(env);
    // variable of controls points, TODO: add conditions
    // n control points and 1 init time
    IloNumVarArray control_points(env, n_points, 0.0, 2000.0);

    // Set start condition
    IloExpr init_place_expr(env);
    std::vector<double> init_bern_val;
    bern_poly.CalculateVal(0, init_bern_val);
    for (int i = 0; i < n_points; i++) {
        init_place_expr += control_points[i]*init_bern_val[i];
    }
    model.add(init_place_expr == 0.0);
    init_place_expr.end();

    // Set end condition
    IloExpr end_place_expr(env);
    std::vector<double> end_bern_val;
    bern_poly.CalculateVal(t_length, end_bern_val);
    for (int i = 0; i < n_points; i++) {
        end_place_expr += control_points[i]*end_bern_val[i];
    }
    model.add(end_place_expr ==
    (traj_size-1)*CELL_DIS);
    end_place_expr.end();

    // Vertex time window constraints
    for (int i = 0; i < traj_size; i++){
        if (i != 0){
            // arrival vertex time
            IloExpr arr_place_expr(env);
            std::vector<double> arr_bern_val;
            bern_poly.CalculateVal(result_nodes[i]->t_min, arr_bern_val);
            for (int j = 0; j < n_points; j++) {
                arr_place_expr += control_points[j]*arr_bern_val[j];
            }
            model.add(arr_place_expr <= (i*CELL_DIS - CELL_DIS/2 - curr_agent.length));
            arr_place_expr.end();
        }
        // leave vertex time
        if (i != (traj_size-1)) {
            IloExpr leave_place_expr(env);
            std::vector<double> leave_bern_val;
            double arr_max = min(result_nodes[i]->t_max, t_length);
            bern_poly.CalculateVal(arr_max, leave_bern_val);
            for (int j = 0; j < n_points; j++) {
                leave_place_expr += control_points[j]*leave_bern_val[j];
            }
            model.add(leave_place_expr >= (i*CELL_DIS + CELL_DIS/2 + curr_agent.length));
            leave_place_expr.end();
        }
    }
    // Add init speed condition
    model.add(control_points[0] == control_points[1]);
    // Add end speed condition
    model.add(control_points[n_points-2] == control_points[n_points-1]);

    // Speed constraints for other conflict points
    for (int i = 1; i <= first_order_deriv.control_points_deriv.N; i++) {
        IloExpr speed_expr(env);
        // For simplicity, we just use 2 here. For any order derivative, use pow(2, first_order_deriv.control_points_deriv.deriv_order) instead
        for (int j = 0; j < 2; j++) {
            speed_expr += first_order_deriv.control_points_deriv.ratio * first_order_deriv.control_points_deriv.sign_idx[i][j]
                          * control_points[first_order_deriv.control_points_deriv.orignal_idx[i][j]];
        }
        model.add(speed_expr <= v_max);
        model.add(speed_expr >= v_min);
        speed_expr.end();
    }

    // Acceleration constraints
    for (int i = 0; i <= second_order_deriv.control_points_deriv.N; i++) {
        IloExpr acce_expr(env);
        // For simplicity, we just use 4 here. For any order derivative, use pow(2, second_order_deriv.control_points_deriv.deriv_order) instead
        for (int j = 0; j < 4; j++) {
            acce_expr += second_order_deriv.control_points_deriv.ratio * second_order_deriv.control_points_deriv.sign_idx[i][j]
                         * control_points[second_order_deriv.control_points_deriv.orignal_idx[i][j]];
        }
        model.add(acce_expr <= a_max);
        model.add(acce_expr >= a_min);
        acce_expr.end();
    }

    IloCplex cplex(model);
    cplex.setOut(env.getNullStream());
    cplex.setWarning(env.getNullStream());
    cplex.setError(env.getNullStream());
    auto startTime = std::chrono::high_resolution_clock::now();
    if(cplex.solve()) {
        auto stopTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime);
        total_runtime_ += duration.count()/1000000.0;
        for (int i = 0; i < n_points; i++) {
            solution_control_points.push_back(cplex.getValue(control_points[i]));
        }

        result_path.clear();
        for (int i = 0; i < traj_size; i++){
            PathEntry a_path;
            // Round all the time slot to TIME_STEP_SIZE
            a_path.location = result_nodes[i]->location;
            if (i == 0) {
                a_path.arrival_time = 0;
            } else {
                double block_start = floor((result_nodes[i]->t_min)/TIME_STEP_SIZE) * TIME_STEP_SIZE  ;
                while((bern_poly.GetVal(block_start, solution_control_points) - EPS) <= (i * CELL_DIS - CELL_DIS/2 - curr_agent.length)
                      and block_start < INF) {
                    block_start += TIME_STEP_SIZE;
                }
                block_start -= TIME_STEP_SIZE;
                a_path.arrival_time = block_start;
            }

            // Only need to modify this
            if (i == traj_size - 1) {
                a_path.leaving_time_tail = floor(T_optimal/TIME_STEP_SIZE)*TIME_STEP_SIZE;
            } else {
                double block_end = a_path.arrival_time;
                while((bern_poly.GetVal(block_end, solution_control_points) + EPS
                       < (i * CELL_DIS + CELL_DIS/2 + curr_agent.length))
                      and block_end < T_optimal) {
                    block_end += TIME_STEP_SIZE;
                }
                if (block_end >= T_optimal) {
                    printf("block end bigger than INF, potential bug!\n");
                    exit(-1);
                }
                a_path.leaving_time_tail = block_end;
            }
            result_path.push_back(a_path);
        }

        env.end();
        return true;
    } else {
        env.end();
        return false;
    }
}


bool TPG_Solver::SolveSlackBezier(
        double T,
        std::vector<double>& solution_control_points,
        Path& result_path,
        double& slack_var)
{
    double a_min = curr_agent.bot_motion->A_MAX * -1.0;
    double a_max = curr_agent.bot_motion->A_MAX;
    double v_min = curr_agent.bot_motion->V_MIN;
    double v_max = curr_agent.bot_motion->V_MAX;
    int traj_size = result_nodes.size();
    double t_length = T;
    BernsteinPolynomial bern_poly(n_points, t_length);
    // First order derivative of bernstein polynomial
    BernsteinPolynomial first_order_deriv = bern_poly.Derivative();
    // Second order derivative of bernstein polynomial
    BernsteinPolynomial second_order_deriv = first_order_deriv.Derivative();

    IloEnv env = IloEnv();
    IloModel model = IloModel(env);
    IloExpr sum_obj = IloExpr(env);
    // variable of controls points, TODO: add conditions
    // n control points and 1 init time
    IloNumVarArray control_points(env, n_points+1, 0, 2000.0);
    sum_obj = control_points[n_points];
    model.add(IloMinimize(env, sum_obj));
    // Set start condition
    IloExpr init_place_expr(env);
    std::vector<double> init_bern_val;
    bern_poly.CalculateVal(0, init_bern_val);
    for (int i = 0; i < n_points; i++) {
        init_place_expr += control_points[i]*init_bern_val[i];
    }
    model.add(init_place_expr == 0.0);
    init_place_expr.end();

    // Set end condition
    IloExpr end_place_expr(env);
    std::vector<double> end_bern_val;
    bern_poly.CalculateVal(t_length, end_bern_val);
    for (int i = 0; i < n_points; i++) {
        end_place_expr += control_points[i]*end_bern_val[i];
    }
    double final_dist = (traj_size - 1)*CELL_DIS;
    model.add(end_place_expr == final_dist);
    end_place_expr.end();
    // Vertex time window constraints
    for (int i = 0; i < traj_size; i++){
        if (i != 0){
            // arrival vertex time
            IloExpr arr_place_expr(env);
            std::vector<double> arr_bern_val;
            bern_poly.CalculateVal(result_nodes[i]->t_min, arr_bern_val);
            for (int j = 0; j < n_points; j++) {
                arr_place_expr += control_points[j]*arr_bern_val[j];
            }
            model.add(arr_place_expr <= (i*CELL_DIS - CELL_DIS/2 - curr_agent.length + control_points[n_points]));
            arr_place_expr.end();
        }
        // leave vertex time
        if (i != (traj_size-1)) {
            IloExpr leave_place_expr(env);
            std::vector<double> leave_bern_val;
            double arr_max = min(result_nodes[i]->t_max, t_length);
            bern_poly.CalculateVal(arr_max, leave_bern_val);
            for (int j = 0; j < n_points; j++) {
                leave_place_expr += control_points[j]*leave_bern_val[j];
            }
            model.add(leave_place_expr >= (i*CELL_DIS + CELL_DIS/2 + curr_agent.length - control_points[n_points]));
            leave_place_expr.end();
        }
    }
    // Add init speed condition
    model.add(control_points[0] == control_points[1]);
    // Add end speed condition
    model.add(control_points[n_points-2] == control_points[n_points-1]);

    // Speed constraints for other conflict points
    for (int i = 1; i <= first_order_deriv.control_points_deriv.N; i++) {
        IloExpr speed_expr(env);
        // For simplicity, we just use 2 here. For any order derivative, use pow(2, first_order_deriv.control_points_deriv.deriv_order) instead
        for (int j = 0; j < 2; j++) {
            speed_expr += first_order_deriv.control_points_deriv.ratio * first_order_deriv.control_points_deriv.sign_idx[i][j]
                          * control_points[first_order_deriv.control_points_deriv.orignal_idx[i][j]];
        }
        model.add(speed_expr <= v_max + control_points[n_points]);
        model.add(speed_expr >= v_min - control_points[n_points]);
        speed_expr.end();
    }

    // Acceleration constraints
    for (int i = 0; i <= second_order_deriv.control_points_deriv.N; i++) {
        IloExpr acce_expr(env);
        // For simplicity, we just use 4 here. For any order derivative, use pow(2, second_order_deriv.control_points_deriv.deriv_order) instead
        for (int j = 0; j < 4; j++) {
            acce_expr += second_order_deriv.control_points_deriv.ratio * second_order_deriv.control_points_deriv.sign_idx[i][j]
                         * control_points[second_order_deriv.control_points_deriv.orignal_idx[i][j]];
        }
        model.add(acce_expr <= a_max + control_points[n_points]);
        model.add(acce_expr >= a_min - control_points[n_points]);
        acce_expr.end();
    }

    IloCplex cplex(model);
    cplex.setOut(env.getNullStream());
    cplex.setWarning(env.getNullStream());
    cplex.setError(env.getNullStream());
    auto startTime = std::chrono::high_resolution_clock::now();
    if(cplex.solve()) {
        auto stopTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime);
        total_runtime_ += duration.count()/1000000.0;
        slack_var = cplex.getValue(control_points[n_points]);
        env.end();
        return true;
    } else {
        env.end();
        return false;
    }
}

bool TPG_Solver::SlackGradient(
        double target_t, double& gradient)
{
    double delta = 0.1;
    std::vector<double> solution_control_points;
    Path result_path;
    double slack_t;
    SolveSlackBezier(target_t, solution_control_points, result_path, slack_t);
    double slack_delta;
    SolveSlackBezier(target_t+delta, solution_control_points, result_path, slack_delta);
    gradient = (slack_delta - slack_t)/delta;
    if (slack_t == 0){
        return true;
    } else {
        return false;
    }
}

void TPG_Solver::retrievePath(std::shared_ptr<MotionNode>& solution, std::shared_ptr<MotionNode>& bcp_solution)
{
    solution = std::make_shared<MotionNode>();
    solution->control_points = bcp_solution->control_points;
    solution->optimal_T = bcp_solution->optimal_T;
    for (size_t i = 0; i < bcp_solution->local_path.size(); i++) {
        solution->local_path.emplace_back(result_nodes[i]->location,
                                          bcp_solution->local_path[i].arrival_time + min_init_t,
                                          bcp_solution->local_path[i].leaving_time_tail + min_init_t);
    }
}

bool TPG_Solver::solve(std::shared_ptr<MotionNode>& solution,
           std::shared_ptr<FailureCache>& failure_cache_ptr,
           std::shared_ptr<SuccessCache>& success_cache_ptr) {
    // Add Cache here. Since we use pointer here, be careful!
    bool found_in_failure = failure_cache_ptr->FindEntry(result_nodes);
    if (found_in_failure or interval_too_small) {
        // No solution for this case
        return false;
    } else {
        std::shared_ptr<MotionNode> bcp_solution;
        bool found_in_success = success_cache_ptr->FindEntry(result_nodes, bcp_solution);
        if (not found_in_success) {
            // No entry in both cache
            bool bcp_success = RecurrentSolver(bcp_solution);
            if (bcp_success) {
                success_cache_ptr->InsertEntry(result_nodes, bcp_solution);
            } else {
                failure_cache_ptr->InsertEntry(result_nodes);
                return false;
            }
        }
        // Retrieve the solution (add init_time)
        retrievePath(solution, bcp_solution);
        return true;
    }
}

bool TPG_Solver::RecurrentSolver(std::shared_ptr<MotionNode>& solution) {
    solution = std::make_shared<MotionNode>();
    Path result_path;
    double lower_bound = result_nodes[result_nodes.size()-1]->t_min;
    double upper_bound = lower_bound;
    double mid;
    double Max_it_thresh = 0.2;
    double gradient_t_low = 0;
    double gradient_t_high = 0;
    double grad_mid = 0;
    bool t_high_valid = false;
    bool upper_in_bould = false;
    double optimal_T;

    bool low_valid_solution = SlackGradient(lower_bound, gradient_t_low);
    bool upper_valid_solution = false;

    if (low_valid_solution) {
        optimal_T = lower_bound;
    } else{
        while (upper_bound < INF) {
            upper_bound = 2 * upper_bound;
            upper_valid_solution = SlackGradient(upper_bound, gradient_t_high);
            if (gradient_t_high > 0 or upper_valid_solution) {
                optimal_T = upper_bound;
                upper_in_bould = true;
                if (upper_valid_solution) {
                    t_high_valid = true;
                }
                break;
            }
        }

        if (!upper_in_bould) {
            return false;
        }

        while ((upper_bound - lower_bound) >= Max_it_thresh) {
            mid = ( (int) (((upper_bound + lower_bound)/2)/TIME_STEP_SIZE) )*TIME_STEP_SIZE;
            double slack;
            SolveSlackBezier(mid, solution->control_points, result_path, slack);
            if (slack == 0) {
                upper_bound = mid;
                optimal_T = mid;
                t_high_valid = true;
            } else {
                if (t_high_valid){
                    lower_bound = mid;
                } else {
                    SlackGradient(mid, grad_mid);
                    if (grad_mid > 0) {
                        upper_bound = mid;
                    } else {
                        lower_bound = mid;
                    }
                }
            }
        }
    }

    if (!t_high_valid) {
        return false;
    }
    solution->control_points.clear();
    bool success = SolveBezier(optimal_T, solution->control_points, result_path);
    if (success){
        solution->optimal_T = optimal_T;
        solution->local_path = result_path;
        return true;
    } else {
        return false;
    }
}

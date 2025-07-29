#include "sipp.h"

using namespace std;

SIPP::SIPP(const std::shared_ptr<Instance>& instance, double cutoff_time,
           shared_ptr<RobotMotion> bot_motion)
    : instance_ptr(instance), cutoff_time(cutoff_time), bot_motion(bot_motion) {
    failure_cache_ptr = std::make_shared<FailureCache>();
    success_cache_ptr = std::make_shared<SuccessCache>();
    // heuristic_vec.resize(instance_ptr->agents.size());
}

void SIPP::PrintNonzeroRT(ReservationTable& rt) const {
    for (unsigned int i = 0; i < rt.size(); i++) {
        if (!rt[i].empty()) {
            printf("Interval at location: %d (%d, %d):\t", i,
                   instance_ptr->graph->getCoordinate(i).first,
                   instance_ptr->graph->getCoordinate(i).second);

            for (TimeInterval tmp_interval : rt[i]) {
                printf("From agent: %d, %f -> %f,\t", tmp_interval.agent_id,
                       tmp_interval.t_min, tmp_interval.t_max);
            }
            printf("\n");
        }
    }
}

bool SIPP::getInitNode(ReservationTable& rt, std::shared_ptr<Node>& init_node,
                       int goal_id) {
    std::vector<TimeInterval> safe_intervals;
    findFreeIntervals(rt[curr_agent.start_location], safe_intervals);
    if (safe_intervals.empty()) {
        // printf("Empty safe initial safe intervals!\n");
        spdlog::error("Empty safe initial safe intervals for agent {}!",
                      curr_agent.id);
        return false;
    } else {
        TimeInterval init_safe_interval;
        for (const auto& interval : safe_intervals) {
            if (interval.t_max >= curr_agent.earliest_start_time) {
                init_safe_interval = interval;
                break;
            }
        }
        assert(safe_intervals[0].t_min <= curr_agent.earliest_start_time);
        if (init_safe_interval.t_min == -1) {
            printf("Invalid safe initial safe intervals!\n");
            return false;
        }
        init_node = std::make_shared<Node>();
        init_node->is_expanded = false;
        init_node->current_point = curr_agent.start_location;
        init_node->curr_o = curr_agent.start_o;
        init_node->interval_index = 0;
        init_node->arrival_time_min = curr_agent.earliest_start_time;
        init_node->arrival_time_max = init_safe_interval.t_max;
        init_node->prev_action = Action::none;
        init_node->goal_id = goal_id;
        // g is the actual time value to that node
        // init_node->g = max(0.0, instance_ptr->simulation_window);
        init_node->g = 0.0;
        // If node is not expanded, h value is the heuristic value. Otherwise,
        // we use the least value in partial interval
        // init_node->h = heuristic_vec[curr_agent.id][init_node->current_point]
        //                             [init_node->curr_o];
        init_node->h = instance_ptr->graph->getHeuristic(
            curr_agent.goal_locations, init_node->current_point,
            init_node->curr_o, init_node->goal_id);
        init_node->f = init_node->g + init_node->h;
        init_node->parent = nullptr;
    }
    return true;
}

void SIPP::Reset() {
    result_nodes.clear();
    closed_set.clear();
    std::priority_queue<std::shared_ptr<Node>,
                        std::vector<std::shared_ptr<Node>>, NodeCompare>()
        .swap(open);
    // allNodes_table.clear();
    useless_nodes.clear();
}

void SIPP::updateResultNodes(std::shared_ptr<Node> res,
                             Path& time_interval_path, MotionInfo& solution,
                             TimedPath& timed_path) {
    solution.clear();
    time_interval_path.clear();
    timed_path.clear();
    if (res->parent == nullptr) {
        // Only one node. We can only let the agent wait at the start
        time_interval_path.emplace_back(res->current_point, res->curr_o,
                                        res->arrival_time_min, INF);
        timed_path.emplace_back(res->current_point,
                                (res->arrival_time_min + INF) / 2.0);
    } else {
        std::shared_ptr<Node> tmp_s = res;
        std::shared_ptr<Node> window_node = nullptr;
        while (tmp_s->parent != nullptr) {
            solution.push_back(tmp_s->bezier_solution);
            if (tmp_s->bezier_solution->start_t < WINDOW_SIZE and
                window_node == nullptr) {
                window_node = tmp_s;
            }
            tmp_s = tmp_s->parent;
        }
        std::reverse(solution.begin(), solution.end());
        assert(not solution.empty());
        int curr_loc = curr_agent.start_location;
        orient curr_o = curr_agent.start_o;
        double curr_time_max = INF;
        double curr_time_min = curr_agent.earliest_start_time;
        for (const auto& motion : solution) {
            assert(motion != nullptr);
            if (motion->type == Action::forward) {
                for (auto local_path_entry : motion->local_path) {
                    if (curr_loc != local_path_entry.location) {
                        time_interval_path.emplace_back(
                            curr_loc, curr_o, curr_time_min, curr_time_max);
                        timed_path.emplace_back(
                            curr_loc, (curr_time_min + curr_time_max) / 2.0);
                        curr_loc = local_path_entry.location;
                        curr_time_min = local_path_entry.arrival_time;
                        curr_time_max = local_path_entry.leaving_time_tail;
                    } else {
                        curr_time_max = local_path_entry.leaving_time_tail;
                    }
                }
            } else {
                curr_o = motion->local_path.front().o;
                curr_time_max = motion->local_path.front().leaving_time_tail;
            }
        }
        assert(window_node != nullptr);
        curr_time_max = INF;
        time_interval_path.emplace_back(curr_loc, curr_o, curr_time_min,
                                        curr_time_max);
        timed_path.emplace_back(curr_loc,
                                (curr_time_min + curr_time_max) / 2.0);
        //    showSolution(res);
    }
}

void SIPP::showSolution(std::shared_ptr<Node>& s) {
    printf("\nFor agent: %d\n", curr_agent.id);
    for (auto local_path_entry : s->bezier_solution->local_path) {
        int curr_loc = local_path_entry.location;
        double curr_time_min = local_path_entry.arrival_time;
        double curr_time_max = local_path_entry.leaving_time_tail;
        printf("Forward::The location (%d, %d, %d), the arrive time is: %f, "
               "the leave time is: %f\n",
               instance_ptr->graph->getCoordinate(curr_loc).first,
               instance_ptr->graph->getCoordinate(curr_loc).second, s->curr_o,
               local_path_entry.arrival_time,
               local_path_entry.leaving_time_tail);
    }
}

bool SIPP::run(int agentID, ReservationTable& rt, MotionInfo& solution,
               Path& path, double& solution_cost, TimedPath& timed_path,
               bool log) {
    this->log = log;
    this->instance_ptr->graph->log = log;
    // Init process
    Reset();
    auto debug_start_t = Time::now();
    count_called++;
    curr_agent = instance_ptr->agents[agentID];
    int goal_id = 0;
    Task curr_goal = curr_agent.goal_locations[goal_id];

    if (log) {
        // Print all goal locations
        cout << "Start to Goals: ";
        cout << "("
             << instance_ptr->graph->getRowCoordinate(curr_agent.start_location)
             << ","
             << instance_ptr->graph->getColCoordinate(curr_agent.start_location)
             << ", " << curr_agent.start_o << ") -> ";
        for (const auto& goal : curr_agent.goal_locations) {
            cout << "(" << instance_ptr->graph->getRowCoordinate(goal.loc)
                 << "," << instance_ptr->graph->getColCoordinate(goal.loc)
                 << ") -> ";
        }
        cout << endl;
    }

    // Skip the goals if they are the same as the start location
    while (goal_id < curr_agent.goal_locations.size() &&
           curr_goal.loc == curr_agent.start_location &&
           (curr_goal.ori == orient::None ||
            curr_goal.ori == curr_agent.start_o)) {
        goal_id++;
        if (goal_id >= curr_agent.goal_locations.size())
            break;
        curr_goal = curr_agent.goal_locations[goal_id];
    }

    if (goal_id >= curr_agent.goal_locations.size()) {
        solution.clear();
        path.clear();
        solution_cost = 0.0;
        if (log)
            spdlog::info("Agent {}: No goals left, returning empty solution.",
                         curr_agent.id);
        return false;
    }

    // Start planning
    path.clear();
    double optimal_travel_time = INF + 1;
    std::shared_ptr<Node> optimal_n = nullptr;

    std::shared_ptr<Node> init_node;
    if (!getInitNode(rt, init_node, goal_id)) {
        path.clear();
        PrintNonzeroRT(rt);
        if (log)
            spdlog::error("Agent {}: Fail to get initial node for SIPP!",
                          curr_agent.id);
        return false;
    }
    open.push(init_node);
    auto start_time = Time::now();
    time_s debug_init_d = start_time - debug_start_t;
    double debug_init_time = debug_init_d.count();
    double debug_retrieve_time = 0;
    double debug_bcp_time = 0;
    double debug_expand_time = 0;
    while (!open.empty()) {
        // remove s with the smallest f-value from OPEN
        auto tmp_end_time = Time::now();
        std::chrono::duration<float> tmp_duration = tmp_end_time - start_time;
        if (tmp_duration.count() > cutoff_time) {
            // printf("Hit cut off time!\n");
            spdlog::info("Agent {}: Hit cutoff time: {}", curr_agent.id,
                         cutoff_time);
            break;
        }
        std::shared_ptr<Node> s = open.top();
        open.pop();
        curr_goal = curr_agent.goal_locations[s->goal_id];

        // Print content of current node
        if (log) {
            spdlog::info(
                "Agent {}: Current node ({},{},{}), goal {}, time: {}, g: {}, "
                "h: "
                "{}, f: {}, prev action: {}, interval index: {}, arrival time "
                "min: {}, arrival time max: {}",
                curr_agent.id,
                instance_ptr->graph->getRowCoordinate(s->current_point),
                instance_ptr->graph->getColCoordinate(s->current_point),
                s->curr_o, s->goal_id, s->arrival_time_min, s->g, s->h, s->f,
                s->prev_action, s->interval_index, s->arrival_time_min,
                s->arrival_time_max);
        }

        if (closed_set.find(s) == closed_set.end()) {
            closed_set.insert(s);
            count_node_closed++;
        } else if (not s->is_expanded) {
            continue;
        }

        // *****************************
        // Found solution, exit the loop
        if (s->f > optimal_travel_time) {
            if (log) {
                spdlog::info(
                    "Agent {} exiting: Current node ({},{},{}), goal {}, time: "
                    "{}, g : {}, h: {}, f: {} is worse than the current "
                    "optimal travel time: {} ",
                    curr_agent.id,
                    instance_ptr->graph->getRowCoordinate(s->current_point),
                    instance_ptr->graph->getColCoordinate(s->current_point),
                    s->curr_o, s->goal_id, s->arrival_time_min, s->g, s->h,
                    s->f, optimal_travel_time);
            }

            break;
        }
        // *****************************

        // Found path to the current goal. Increment the goal_id and check if
        // we have reached the last goal.
        bool reached_goal = false;
        if (s->current_point == curr_goal.loc and
            (curr_goal.ori == orient::None or s->curr_o == curr_goal.ori)) {
            // For the last goal, we must make sure we can hold it forever (in
            // non-windowed case). Do not increment goal_id if we cannot hold
            // it.
            // Check if we reached the last goal, and there is no target
            // conflicts. We need to check target conflict in both windowed and
            // non-windowed cases.
            if (s->goal_id + 1 == curr_agent.goal_locations.size()) {
                if (s->arrival_time_max < INF) {
                    if (log) {
                        spdlog::info("Agent {}: Reached last goal {}, time: "
                                     "{},g: {}, h: {}, f: {}, but cannot hold.",
                                     curr_agent.id, s->goal_id, s->g, s->g,
                                     s->h, s->f);
                    }
                } else {
                    reached_goal = true;
                }
            }
            // Otherwise, we proceed the goal id
            else {
                // spdlog::info(
                //     "Agent {}: Reached goal {}, time: {}, g: {}, h: {}, f:
                //     {}", curr_agent.id, s->goal_id, s->g, s->g, s->h, s->f);
                // Move to the next goal
                std::shared_ptr<Node> new_n = std::make_shared<Node>(
                    s->goal_id + 1, s->current_point, s->curr_o,
                    s->interval_index, s->arrival_time_min, s->arrival_time_max,
                    s->arrival_time_min,
                    instance_ptr->graph->getHeuristic(
                        curr_agent.goal_locations, s->current_point, s->curr_o,
                        s->goal_id + 1),
                    instance_ptr->simulation_window, s->prev_action, s->parent,
                    s->bezier_solution);
                pushToOpen(new_n);
            }
        }

        // Reached all goals
        if (reached_goal) {
            optimal_travel_time = s->g;
            optimal_n = s;
            if (log) {
                spdlog::info("Agent {}: Found optimal solution with: g: {}, h: "
                             "{}, f: {}",
                             curr_agent.id, s->g, s->h, s->f);
            }

            // NOTE: To get optimal solution we should `continue` here. By
            // `break`, we stop the search when the first solution is found.
            // This can speed up the search.
            break;
        }

        // // The current f val is smaller than the current optimal (in windowed
        // // case), we update the current optimal
        // if (instance_ptr->simulation_window > 0 && s->arrival_time_max == INF
        // &&
        //     s->f < optimal_travel_time) {
        //     optimal_travel_time = s->f;
        //     optimal_n = s;
        //     spdlog::info(
        //         "Agent {}: Found better solution at ({},{}) with: "
        //         "{}, g: {}, h: {}, f: {}",
        //         agentID,
        //         instance_ptr->graph->getRowCoordinate(s->current_point),
        //         instance_ptr->graph->getColCoordinate(s->current_point),
        //         curr_agent.id, s->g, s->h, s->f);
        // }

        // Not reached all goals. Expand the node if it has not reached the
        // window, or if there is no window.
        // if (instance_ptr->simulation_window <= 0) {
        else {
            // spdlog::info(
            //     "Agent {}: Expanding node ({},{}), goal {}, time: {}, g: {},
            //     " "h: "
            //     "{}, f: {}, prev action: {}, interval index: {}, arrival time
            //     " "min: {}, arrival time max: {}", curr_agent.id,
            //     instance_ptr->graph->getRowCoordinate(s->current_point),
            //     instance_ptr->graph->getColCoordinate(s->current_point),
            //     s->goal_id, s->arrival_time_min, s->g, s->h, s->f,
            //     s->prev_action, s->interval_index, s->arrival_time_min,
            //     s->arrival_time_max);
            // Node has been expanded before. Pop the next reachable interval
            // and do create a movement node (Line 9-10 of Algorithm 2)
            if (s->is_expanded) {
                if (log) {
                    spdlog::info(
                        "Agent {}: Node ({},{},{}) was expanded before.",
                        curr_agent.id,
                        instance_ptr->graph->getRowCoordinate(s->current_point),
                        instance_ptr->graph->getColCoordinate(s->current_point),
                        s->curr_o);
                }
                count_node_re_expand++;
                auto debug_expand_start_t = Time::now();
                while (not s->partial_intervals.empty()) {
                    std::shared_ptr<IntervalEntry> tmp_min_entry =
                        s->partial_intervals.top();
                    s->partial_intervals.pop();
                    // Retrieve the path
                    std::deque<std::shared_ptr<IntervalEntry>>
                        forward_intervals;
                    std::shared_ptr<IntervalEntry> prev_entry = tmp_min_entry;
                    while (prev_entry != nullptr) {
                        forward_intervals.push_front(prev_entry);
                        prev_entry = prev_entry->prev_entry;
                    }

                    if (log) {
                        spdlog::info("Agent {}: Expanding node ({},{},{}) with "
                                     "interval: loc={},t_min={},t_max={}",
                                     curr_agent.id,
                                     instance_ptr->graph->getRowCoordinate(
                                         s->current_point),
                                     instance_ptr->graph->getColCoordinate(
                                         s->current_point),
                                     s->curr_o, tmp_min_entry->location,
                                     tmp_min_entry->t_min,
                                     tmp_min_entry->t_max);
                    }

                    bool success;
                    auto debug_bcp_start_t = Time::now();
                    std::shared_ptr<MotionNode> tpg_solution;

                    // Use SPS solver to compute the speed profile.
                    // Using BCS solver
                    if (instance_ptr->use_sps_type == 1) {
                        TPG_Solver tpg_solver(curr_agent, forward_intervals);
                        time_s debug_retrieve_d =
                            debug_bcp_start_t - debug_expand_start_t;
                        debug_retrieve_time += debug_retrieve_d.count();
                        success = tpg_solver.solve(
                            tpg_solution, failure_cache_ptr, success_cache_ptr);
                    }
                    //                    else if (instance_ptr->use_sps_type ==
                    //                    2) {
                    //                        SIPP_IP sipp_ip_solver(curr_agent,
                    //                        forward_intervals, s->curr_o);
                    //                        time_s debug_retrieve_d =
                    //                        debug_bcp_start_t -
                    //                        debug_expand_start_t;
                    //                        debug_retrieve_time +=
                    //                        debug_retrieve_d.count(); success
                    //                        =
                    //                        sipp_ip_solver.solve(tpg_solution);
                    //                    }
                    // Using BAS solver
                    else {
                        CRISE_Solver crise_solver(curr_agent, forward_intervals,
                                                  s->curr_o, bot_motion);
                        time_s debug_retrieve_d =
                            debug_bcp_start_t - debug_expand_start_t;
                        debug_retrieve_time += debug_retrieve_d.count();
                        success = crise_solver.solve(tpg_solution);
                    }

                    auto debug_bcp_finish_t = Time::now();
                    time_s debug_bcp_d = debug_bcp_finish_t - debug_bcp_start_t;
                    debug_bcp_time += debug_bcp_d.count();

                    // Successfully computed the speed profile. Create the new
                    // node
                    if (success) {
                        std::shared_ptr<Node> new_node =
                            std::make_shared<Node>();
                        new_node->prev_action = Action::forward;
                        new_node->current_point = tmp_min_entry->location;
                        new_node->curr_o = s->curr_o;
                        new_node->interval_index = tmp_min_entry->interval_idx;
                        new_node->arrival_time_min =
                            tpg_solution->local_path.back().leaving_time_tail;
                        new_node->arrival_time_max = tmp_min_entry->t_max;
                        // TODO: Change the computation of g, h to weighted sum
                        // of edge weights and travel time.
                        // if (instance_ptr->simulation_window <= 0)
                        new_node->g = new_node->arrival_time_min;
                        // else
                        //     new_node->g = max(new_node->arrival_time_min,
                        //                       instance_ptr->simulation_window);
                        // new_node->h = heuristic_vec[curr_agent.id]
                        //                            [new_node->current_point]
                        //                            [new_node->curr_o];
                        new_node->h = instance_ptr->graph->getHeuristic(
                            curr_agent.goal_locations, new_node->current_point,
                            new_node->curr_o, s->goal_id);
                        new_node->f = new_node->g + new_node->h;
                        tpg_solution->start_t =
                            tpg_solution->local_path.front().arrival_time;
                        tpg_solution->end_t =
                            tpg_solution->local_path.back().leaving_time_tail;
                        tpg_solution->type = Action::forward;
                        new_node->bezier_solution = tpg_solution;
                        new_node->parent = s;
                        new_node->goal_id = s->goal_id;
                        if (log) {
                            spdlog::info(
                                "Agent {}: Created new node ({},{},{}), "
                                "goal {}, time: {}, g: {}, h: {}, f: {}, "
                                "prev action: {}, interval index: {}, "
                                "arrival time min: {}, arrival time max: {}",
                                curr_agent.id,
                                instance_ptr->graph->getRowCoordinate(
                                    new_node->current_point),
                                instance_ptr->graph->getColCoordinate(
                                    new_node->current_point),
                                new_node->curr_o, new_node->goal_id,
                                new_node->arrival_time_min, new_node->g,
                                new_node->h, new_node->f, new_node->prev_action,
                                new_node->interval_index,
                                new_node->arrival_time_min,
                                new_node->arrival_time_max);
                        }
                        pushToOpen(new_node);
                    } else {
                        if (log)
                            spdlog::info("speed profile is not found");
                    }

                    // If we are using partial expansion, we can stop creating
                    // new nodes from other reachable intervals.
                    if (instance_ptr->use_pe) {
                        break;
                    }
                }
            }
            // This node has not been expanded before. We need to expand it by:
            // 1. doing rotational expandsions if necessary
            // 2. finding the reachable intervals and sort the intervals by
            //    their p values
            // Corresponds to line 2 to 8 of Algorithm 2
            else {
                count_node_expanded++;
                auto debug_expand_start_t = Time::now();
                nodeExpansion(s, rt);
                auto debug_expand_end_t = Time::now();
                time_s debug_expand_d =
                    debug_expand_end_t - debug_expand_start_t;
                debug_expand_time += debug_expand_d.count();
            }
            assert(s->is_expanded);

            // Print partial intervals
            if (log) {
                spdlog::info(
                    "Agent {}: Node ({},{},{}) has {} partial intervals after "
                    "expansion.",
                    curr_agent.id,
                    instance_ptr->graph->getRowCoordinate(s->current_point),
                    instance_ptr->graph->getColCoordinate(s->current_point),
                    s->curr_o, s->partial_intervals.size());
            }

            // If not all the reachable intervals have been expanded,
            // we need to update the heuristic value and push it back to OPEN.
            if (not s->partial_intervals.empty()) {
                // update heuristic for s
                s->h = s->partial_intervals.top()->f;

                // One-shot MASS objective
                // if (instance_ptr->simulation_window <= 0)
                s->f = s->g + s->h;
                // // Windowed MASS objective
                // else
                //     s->f = max(s->g, instance_ptr->simulation_window) + s->h;
                open.push(s);
                count_node_generated++;
            }
        }
        // } else {
        //     spdlog::info(
        //         "Agent {}: Exceeded the simulation window for ({},{}), "
        //         "current arrival time: {}, window size: {}",
        //         curr_agent.id,
        //         instance_ptr->graph->getRowCoordinate(s->current_point),
        //         instance_ptr->graph->getColCoordinate(s->current_point),
        //         s->arrival_time_min, instance_ptr->simulation_window);
        // }
    }
    auto debug_end_t = Time::now();
    time_s total_runtime_t = debug_end_t - debug_start_t;
    total_runtime_ += total_runtime_t.count();
    init_runtime += debug_init_time;
    retrieve_runtime += debug_retrieve_time;
    motion_solver_runtime += debug_bcp_time;
    expand_runtime += debug_expand_time;

    if (optimal_n == nullptr) {
        if (log)
            spdlog::info(
                "Agent {}: No solution found, current optimal travel time: {}",
                curr_agent.id, optimal_travel_time);
        return false;
    } else {
        if (log)
            spdlog::info("Agent {}: Found solution with optimal travel time: "
                         "{}, g: {}, h:"
                         "{}, f: {}",
                         curr_agent.id, optimal_travel_time, optimal_n->g,
                         optimal_n->h, optimal_n->f);
        solution_cost = optimal_travel_time;
        updateResultNodes(optimal_n, path, solution, timed_path);
        return true;
    }
}

// // return true iff we the new node is not dominated by any old node
// bool SIPP::dominanceCheck(const std::shared_ptr<Node>& new_node) {
//     auto ptr = allNodes_table.find(new_node);
//     if (ptr == allNodes_table.end())
//         return true;
//     for (auto& old_node : ptr->second) {
//         if ((old_node->g - new_node->g) <
//             EPS) {  // the new node is dominated by the old node
//             return false;
//         } else  // the old node is dominated by the new node
//         {       // delete the old node
//             useless_nodes.insert(old_node);
//             ptr->second.remove(old_node);
//             count_node_generated--;  // this is because we later will
//             increase
//                                      // num_generated when we insert the
//                                      new
//                                      // node into lists.
//             return true;
//         }
//     }
//     return true;
// }

/**
 * @brief Expand the node that has never been expand before
 *
 * @param n The node that need to be expanded
 * @param rt The reservation table
 */
void SIPP::nodeExpansion(const std::shared_ptr<Node>& n, ReservationTable& rt) {
    Neighbors neighbors;
    instance_ptr->graph->getNeighbors(n->current_point, n->curr_o, neighbors);
    // Previous action is forward. Do rotation expansion here.
    if (log) {
        spdlog::info(
            "Expanding node ({},{},{}), goal {}, time: {}, g: {}, h: {}, "
            "f: {}, prev action: {}, interval index: {}, arrival time "
            "min: {}, arrival time max: {}",
            instance_ptr->graph->getRowCoordinate(n->current_point),
            instance_ptr->graph->getColCoordinate(n->current_point), n->curr_o,
            n->goal_id, n->arrival_time_min, n->g, n->h, n->f, n->prev_action,
            n->interval_index, n->arrival_time_min, n->arrival_time_max);

        // Print neighbors
        spdlog::info("Neighbors:");
        spdlog::info(
            "Left: ({},{}, {})",
            instance_ptr->graph->getRowCoordinate(neighbors.left.first),
            instance_ptr->graph->getColCoordinate(neighbors.left.first),
            neighbors.left.second);
        spdlog::info(
            "Right: ({},{}, {})",
            instance_ptr->graph->getRowCoordinate(neighbors.right.first),
            instance_ptr->graph->getColCoordinate(neighbors.right.first),
            neighbors.right.second);
        spdlog::info(
            "Back: ({},{}, {})",
            instance_ptr->graph->getRowCoordinate(neighbors.back.first),
            instance_ptr->graph->getColCoordinate(neighbors.back.first),
            neighbors.back.second);
        spdlog::info("Forward:");
        for (const auto& forward : neighbors.forward_locs) {
            cout << "(" << instance_ptr->graph->getRowCoordinate(forward)
                 << ", " << instance_ptr->graph->getColCoordinate(forward)
                 << "),  ";
        }
        cout << endl;
    }
    if (n->parent == nullptr or n->prev_action == Action::forward) {
        // Insert three nodes, turn left, turn right, and turn back
        if ((n->arrival_time_min + bot_motion->ROTATE_COST) <
            n->arrival_time_max) {
            std::shared_ptr<Node> n_left = std::make_shared<Node>();
            std::shared_ptr<MotionNode> motion = std::make_shared<MotionNode>();
            motion->start_t = n->arrival_time_min;
            motion->end_t = n->arrival_time_min + bot_motion->ROTATE_COST;
            motion->local_path.emplace_back(n->current_point,
                                            neighbors.left.second,
                                            motion->start_t, motion->end_t);
            motion->type = Action::turnLeft;
            n_left->bezier_solution = motion;
            n_left->is_expanded = false;
            n_left->prev_action = Action::turnLeft;
            n_left->current_point = n->current_point;
            n_left->curr_o = neighbors.left.second;
            n_left->interval_index = n->interval_index;
            n_left->arrival_time_min =
                n->arrival_time_min + bot_motion->ROTATE_COST;
            n_left->arrival_time_max = n->arrival_time_max;
            // if (instance_ptr->simulation_window <= 0)
            n_left->g = n_left->arrival_time_min;
            // else
            //     n_left->g = max(n_left->arrival_time_min,
            //                     instance_ptr->simulation_window);
            // n_left->h =
            // heuristic_vec[curr_agent.id][n_left->current_point]
            //                          [n_left->curr_o];
            n_left->h = instance_ptr->graph->getHeuristic(
                curr_agent.goal_locations, n_left->current_point,
                n_left->curr_o, n->goal_id);
            n_left->f = n_left->g + n_left->h;
            n_left->parent = n;
            n_left->goal_id = n->goal_id;
            pushToOpen(n_left);
            // Print g and h values
            if (log) {
                spdlog::info(
                    "Agent {}: Created left turn node ({},{},{}), goal {}, "
                    "time: {}, g: {}, h: {}, f: {}, prev action: {}, "
                    "interval index: {}, arrival time min: {}, arrival time "
                    "max: {}",
                    curr_agent.id,
                    instance_ptr->graph->getRowCoordinate(
                        n_left->current_point),
                    instance_ptr->graph->getColCoordinate(
                        n_left->current_point),
                    n_left->curr_o, n_left->goal_id, n_left->arrival_time_min,
                    n_left->g, n_left->h, n_left->f, n_left->prev_action,
                    n_left->interval_index, n_left->arrival_time_min,
                    n_left->arrival_time_max);
            }
        }

        if ((n->arrival_time_min + bot_motion->ROTATE_COST) <
            n->arrival_time_max) {
            std::shared_ptr<Node> n_right = std::make_shared<Node>();
            std::shared_ptr<MotionNode> motion = std::make_shared<MotionNode>();
            motion->start_t = n->arrival_time_min;
            motion->end_t = n->arrival_time_min + bot_motion->ROTATE_COST;
            motion->type = Action::turnRight;
            motion->local_path.emplace_back(n->current_point,
                                            neighbors.right.second,
                                            motion->start_t, motion->end_t);
            n_right->bezier_solution = motion;
            n_right->is_expanded = false;
            n_right->prev_action = Action::turnRight;
            n_right->current_point = n->current_point;
            n_right->curr_o = neighbors.right.second;
            n_right->interval_index = n->interval_index;
            n_right->arrival_time_min =
                n->arrival_time_min + bot_motion->ROTATE_COST;
            n_right->arrival_time_max = n->arrival_time_max;
            // if (instance_ptr->simulation_window <= 0)
            n_right->g = n_right->arrival_time_min;
            // else
            //     n_right->g = max(n_right->arrival_time_min,
            //                      instance_ptr->simulation_window);
            // n_right->h =
            // heuristic_vec[curr_agent.id][n_right->current_point]
            //                           [n_right->curr_o];
            n_right->h = instance_ptr->graph->getHeuristic(
                curr_agent.goal_locations, n_right->current_point,
                n_right->curr_o, n->goal_id);
            n_right->f = n_right->g + n_right->h;
            n_right->parent = n;
            n_right->goal_id = n->goal_id;
            pushToOpen(n_right);

            if (log) {
                spdlog::info(
                    "Agent {}: Created right turn node ({},{},{}), goal {}, "
                    "time: {}, g: {}, h: {}, f: {}, prev action: {}, "
                    "interval index: {}, arrival time min: {}, arrival time "
                    "max: {}",
                    curr_agent.id,
                    instance_ptr->graph->getRowCoordinate(
                        n_right->current_point),
                    instance_ptr->graph->getColCoordinate(
                        n_right->current_point),
                    n_right->curr_o, n_right->goal_id,
                    n_right->arrival_time_min, n_right->g, n_right->h,
                    n_right->f, n_right->prev_action, n_right->interval_index,
                    n_right->arrival_time_min, n_right->arrival_time_max);
            }
        }

        if ((n->arrival_time_min + bot_motion->TURN_BACK_COST) <
            n->arrival_time_max) {
            std::shared_ptr<Node> n_back = std::make_shared<Node>();
            std::shared_ptr<MotionNode> motion = std::make_shared<MotionNode>();
            motion->start_t = n->arrival_time_min;
            motion->end_t = n->arrival_time_min + bot_motion->TURN_BACK_COST;
            motion->type = Action::turnBack;
            motion->local_path.emplace_back(n->current_point,
                                            neighbors.back.second,
                                            motion->start_t, motion->end_t);
            n_back->bezier_solution = motion;
            n_back->is_expanded = false;
            n_back->prev_action = Action::turnBack;
            n_back->current_point = n->current_point;
            n_back->curr_o = neighbors.back.second;
            n_back->interval_index = n->interval_index;
            n_back->arrival_time_min =
                n->arrival_time_min + bot_motion->TURN_BACK_COST;
            n_back->arrival_time_max = n->arrival_time_max;
            // if (instance_ptr->simulation_window <= 0)
            n_back->g = n_back->arrival_time_min;
            // else
            //     n_back->g = max(n_back->arrival_time_min,
            //                     instance_ptr->simulation_window);
            // n_back->h =
            // heuristic_vec[curr_agent.id][n_back->current_point]
            //                          [n_back->curr_o];
            n_back->h = instance_ptr->graph->getHeuristic(
                curr_agent.goal_locations, n_back->current_point,
                n_back->curr_o, n->goal_id);
            n_back->f = n_back->g + n_back->h;
            n_back->parent = n;
            n_back->goal_id = n->goal_id;
            pushToOpen(n_back);

            if (log) {
                spdlog::info(
                    "Agent {}: Created back turn node ({},{},{}), goal {}, "
                    "time: {}, g: {}, h: {}, f: {}, prev action: {}, "
                    "interval index: {}, arrival time min: {}, arrival time "
                    "max: {}",
                    curr_agent.id,
                    instance_ptr->graph->getRowCoordinate(
                        n_back->current_point),
                    instance_ptr->graph->getColCoordinate(
                        n_back->current_point),
                    n_back->curr_o, n_back->goal_id, n_back->arrival_time_min,
                    n_back->g, n_back->h, n_back->f, n_back->prev_action,
                    n_back->interval_index, n_back->arrival_time_min,
                    n_back->arrival_time_max);
            }
        }
    }

    // Previous action is not forward. Do movement expansion from node n.
    // Nodes are not allowed to have two consecutive move forward
    if (n->parent == nullptr or n->prev_action != Action::forward) {
        getSuccessors(n, neighbors.forward_locs, rt);
        count_potential_interval += n->partial_intervals.size();
    }

    n->is_expanded = true;
}

void getShape(
    const std::vector<std::vector<std::vector<std::vector<double>>>>& vec) {
    size_t dim1 = vec.size();
    size_t dim2 = dim1 > 0 ? vec[0].size() : 0;
    size_t dim3 = (dim1 > 0 && dim2 > 0) ? vec[0][0].size() : 0;
    size_t dim4 = (dim1 > 0 && dim2 > 0 && dim3 > 0) ? vec[0][0][0].size() : 0;

    std::cout << "Shape: [" << dim1 << ", " << dim2 << ", " << dim3 << ", "
              << dim4 << "]" << std::endl;
}

void save3DVector(const std::vector<std::vector<std::vector<double>>>& vec,
                  const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file for writing");
    }

    for (const auto& dim1 : vec) {
        for (const auto& dim2 : dim1) {
            for (double value : dim2) {
                file << value << ' ';
            }
            file << '\n';  // Separate each 3rd dimension with a newline
        }
        file << "\n";  // Separate each 1st dimension with three newlines
    }
    file.close();
}

std::vector<std::vector<std::vector<double>>> loadVector(
    const std::string& filename) {
    std::vector<std::vector<std::vector<double>>> vec;
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file for reading");
    }

    std::string line;
    std::vector<std::vector<double>> dim1;
    std::vector<double> dim2;

    while (getline(file, line)) {
        if (line.empty()) {
            if (!dim2.empty()) {
                dim1.push_back(dim2);
                dim2.clear();
            } else if (!dim1.empty()) {
                vec.push_back(dim1);
                dim1.clear();
            }
        } else {
            std::istringstream iss(line);
            double value;
            while (iss >> value) {
                dim2.push_back(value * 10);
            }
            dim1.push_back(dim2);
            dim2.clear();
        }
    }

    if (!dim2.empty()) {
        dim1.push_back(dim2);
    }
    if (!dim1.empty()) {
        vec.push_back(dim1);
    }

    return vec;
}

// void SIPP::getHeuristic(const std::string& heuristic_file) {
//     ifstream myfile(heuristic_file.c_str());
//     if (myfile.is_open()) {
//         printf("success in loading heuristic!\n");

//         heuristic_vec = loadVector(heuristic_file);
//     } else {
//         printf("Start to compute global heuristic!\n");
//         size_t numCores = std::thread::hardware_concurrency();
//         numCores = min(5, (int)numCores);
//         for (size_t i = 0; i < instance_ptr->num_of_agents; i +=
//         numCores) {
//             std::vector<std::thread> threads;
//             // Pass member function and object reference
//             for (size_t j = i;
//                  j < min(instance_ptr->num_of_agents, (int)(i +
//                  numCores)); j++) {
//                 threads.emplace_back(&SIPP::Dijkstra, this, j);
//             }
//             for (auto& thread : threads) {
//                 thread.join();
//             }
//         }
//     }
// }

// /**
//  * @brief Help function, get the heuristic values
//  *
//  * @param start_loc The start location of the agent
//  * @return Bool value determine if the search success
//  */
// bool SIPP::Dijkstra(size_t curr_id) {
//     std::vector<std::vector<double>> curr_agent_heuristic(
//         instance_ptr->graph->map_size, std::vector<double>(NUM_ORIENT));
//     std::priority_queue<std::shared_ptr<Node>,
//                         std::vector<std::shared_ptr<Node>>, NodeCompare>
//         dij_open;
//     std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual>
//         dij_close_set;
//     std::shared_ptr<Node> root_node = std::make_shared<Node>();
//     root_node->g = 0.0;
//     root_node->f = 0.0;
//     root_node->current_point =
//     instance_ptr->agents[curr_id].goal_location; root_node->curr_o =
//     instance_ptr->agents[curr_id].end_o; root_node->parent = nullptr;
//     dij_open.push(root_node);
//     size_t h_count = 0;
//     while (!dij_open.empty()) {
//         std::shared_ptr<Node> n = dij_open.top();
//         dij_open.pop();
//         auto close_item_it = dij_close_set.find(n);
//         if (close_item_it != dij_close_set.end()) {
//             if (close_item_it->get()->f <= n->f) {
//                 continue;
//             } else {
//                 dij_close_set.erase(close_item_it);
//                 curr_agent_heuristic[n->current_point][n->curr_o] = n->g
//                 * 2; dij_close_set.insert(n);
//             }
//         } else {
//             assert(n->current_point < curr_agent_heuristic.size());
//             assert(n->curr_o < curr_agent_heuristic[0].size());
//             curr_agent_heuristic[n->current_point][n->curr_o] = n->g * 2;
//             h_count++;
//             dij_close_set.insert(n);
//         }
//         // For all the neighbor location, all need to do this operation
//         Neighbors neighbors;
//         instance_ptr->graph->getInverseNeighbors(n->current_point,
//         n->curr_o,
//                                           neighbors);
//         if (n->parent == nullptr or n->prev_action == Action::forward) {
//             // Insert two nodes, turn left and turn right
//             std::shared_ptr<Node> n_left = std::make_shared<Node>();
//             n_left->is_expanded = false;
//             n_left->prev_action = Action::turnLeft;
//             n_left->current_point = n->current_point;
//             n_left->curr_o = neighbors.left.second;
//             n_left->g = n->g + bot_motion->ROTATE_COST;
//             n_left->f = n_left->g;
//             n_left->parent = n;
//             dij_open.push(n_left);

//             std::shared_ptr<Node> n_right = std::make_shared<Node>();
//             n_right->is_expanded = false;
//             n_right->prev_action = Action::turnRight;
//             n_right->current_point = n->current_point;
//             n_right->curr_o = neighbors.right.second;
//             n_right->g = n->g + bot_motion->ROTATE_COST;
//             n_right->f = n_right->g;
//             n_right->parent = n;
//             dij_open.push(n_right);

//             std::shared_ptr<Node> n_back = std::make_shared<Node>();
//             n_back->is_expanded = false;
//             n_back->prev_action = Action::turnBack;
//             n_back->current_point = n->current_point;
//             n_back->curr_o = neighbors.back.second;
//             n_back->g = n->g + bot_motion->TURN_BACK_COST;
//             n_back->f = n_back->g;
//             n_back->parent = n;
//             dij_open.push(n_back);
//         }

//         for (int i = 0; i < neighbors.forward_locs.size(); i++) {
//             std::shared_ptr<Node> n_back = std::make_shared<Node>();
//             n_back->is_expanded = false;
//             n_back->prev_action = Action::forward;
//             n_back->current_point = neighbors.forward_locs[i];
//             n_back->curr_o = n->curr_o;
//             n_back->g = n->g + arrLowerBound(i);
//             n_back->f = n_back->g;
//             n_back->parent = n;
//             dij_open.push(n_back);
//         }
//     }
//     std::lock_guard<std::mutex> lock(mutex);
//     heuristic_vec[curr_id] = curr_agent_heuristic;
//     return false;
// }

/**
 * @brief Find next free intervals recurrently
 *
 * @param prev_interval The time interval to be expanded
 * @param next_locs The potential locations that can be reached from the
 * expanded node
 * @param rt The reservation table
 * @param[out] new_interval_list A priority queue contains all the time
 * intervals
 * */
void SIPP::GetNextInterval(const std::shared_ptr<IntervalEntry>& prev_interval,
                           IntervalQueue& new_interval_list,
                           std::vector<int>& next_locs, ReservationTable& rt,
                           const std::shared_ptr<Node>& s) {
    // Base case, if the step is larger than the number of next locations,
    if (next_locs.size() <= prev_interval->step)
        return;
    int next_loc = next_locs[prev_interval->step];
    // Compute t_min as the minimum time to travel to the next location
    // TODO: Split t_min into time of waiting and time of moving. Then
    // compute weighted t_min
    double m_time_min = travel_cost[min((int)prev_interval->step, 4)];
    // Compute lower bound as lb + t_min
    double lower_bound = prev_interval->t_min + m_time_min;
    std::vector<TimeInterval> interval_list;
    findFreeIntervals(rt[next_loc], interval_list);
    for (TimeInterval interval : interval_list) {
        // If lower bound is bigger than t_max of candidate interval, the
        // candidate interval is invalid If t_min of candidate interval is
        // larger than t_max of the previous interval, no transition exists
        // for such a case
        if (lower_bound >= interval.t_max or
            prev_interval->t_max <= interval.t_min)
            continue;

        // the interval overlaps with [lower_bound, upper_bound)
        std::shared_ptr<IntervalEntry> new_interval =
            std::make_shared<IntervalEntry>();
        new_interval->interval_idx = interval.id;
        new_interval->t_min = max(lower_bound, interval.t_min);
        new_interval->t_max = min(INF, interval.t_max);
        // Compute p value as the lower bound plus the heuristic value
        // TODO: Change to weighted p-val
        // If the lb of next interval (lb1 = interval.t_min) is larger than
        // lower_bound = lb0 + m_time_min, the agent will implicitly
        // wait at previous interval for (lb1 - (lb0 + m_time_min)) time
        // old: buggy because it does not consider the waiting time
        // new_interval->f = arrLowerBound(prev_interval->step) +
        //                   heuristic_vec[curr_agent.id][next_loc][s->curr_o];
        // new_interval->f = new_interval->t_min +
        //                   heuristic_vec[curr_agent.id][next_loc][s->curr_o];
        new_interval->f =
            new_interval->t_min +
            instance_ptr->graph->getHeuristic(curr_agent.goal_locations,
                                              next_loc, s->curr_o, s->goal_id);
        new_interval->location = next_loc;
        new_interval->step = prev_interval->step + 1;
        new_interval->prev_entry = prev_interval;
        GetNextInterval(new_interval, new_interval_list, next_locs, rt, s);
        if (log) {
            spdlog::info(
                "Agent {}: Found new interval with step {}, location {}, "
                "t_min {}, t_max {}, f {}",
                curr_agent.id, new_interval->step, new_interval->location,
                new_interval->t_min, new_interval->t_max, new_interval->f);
        }
        new_interval_list.push(new_interval);
    }
}

/**
 * @brief Get the potential intervals that can be obtained from moving
 * forward
 *
 * @param s The node need to be expanded
 * @param to_locs The potential locations that can be reached from the
 * expanded node
 * @param rt The reservation table
 * */
void SIPP::getSuccessors(const std::shared_ptr<Node>& s,
                         std::vector<int>& to_locs, ReservationTable& rt) {
    std::vector<int> locations;
    IntervalQueue new_interval_list;
    // Create first interval
    std::shared_ptr<IntervalEntry> init_interval =
        std::make_shared<IntervalEntry>();
    init_interval->step = 0;
    init_interval->t_min = s->arrival_time_min;
    init_interval->t_max = s->arrival_time_max;
    // init_interval->f =
    //     heuristic_vec[curr_agent.id][s->current_point][s->curr_o];
    init_interval->f = instance_ptr->graph->getHeuristic(
        curr_agent.goal_locations, s->current_point, s->curr_o, s->goal_id);
    init_interval->prev_entry = nullptr;
    init_interval->location = s->current_point;
    init_interval->interval_idx = s->interval_index;
    GetNextInterval(init_interval, new_interval_list, to_locs, rt, s);
    s->partial_intervals = new_interval_list;
}

/**
 * @brief Find free intervals given the reserved intervals
 *
 * @param reservations Reservation table
 * @return The free intervals
 * */
void SIPP::findFreeIntervals(std::vector<TimeInterval>& reservations,
                             std::vector<TimeInterval>& freeIntervals) {
    // Sort reservations by start time
    std::sort(reservations.begin(), reservations.end(),
              [](const TimeInterval& a, const TimeInterval& b) {
                  return a.t_min < b.t_min;
              });
    double current_time = 0.0;
    int interval_id = 0;
    for (const TimeInterval& reservation : reservations) {
        if (reservation.t_min > current_time) {
            freeIntervals.emplace_back(current_time, reservation.t_min,
                                       interval_id);
        }
        interval_id++;
        current_time = std::max(current_time, reservation.t_max);
    }
    if (current_time < INF) {
        freeIntervals.emplace_back(current_time, INF, interval_id);
    }
}
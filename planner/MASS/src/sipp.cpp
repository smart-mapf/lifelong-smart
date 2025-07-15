#include "sipp.h"

using namespace std;

SIPP::SIPP(const std::shared_ptr<Instance>& instance, double cutoff_time)
    : instance_ptr(instance), cutoff_time(cutoff_time) {
    failure_cache_ptr = std::make_shared<FailureCache>();
    success_cache_ptr = std::make_shared<SuccessCache>();
    heuristic_vec.resize(instance_ptr->agents.size());
}

void SIPP::PrintNonzeroRT(ReservationTable& rt) const {
    for (unsigned int i = 0; i < rt.size(); i++) {
        if (!rt[i].empty()) {
            printf("Interval at location: %d (%d, %d):\t", i,
                   instance_ptr->getCoordinate(i).first,
                   instance_ptr->getCoordinate(i).second);

            for (TimeInterval tmp_interval : rt[i]) {
                printf("From agent: %d, %f -> %f,\t", tmp_interval.agent_id,
                       tmp_interval.t_min, tmp_interval.t_max);
            }
            printf("\n");
        }
    }
}

bool SIPP::getInitNode(ReservationTable& rt, std::shared_ptr<Node>& init_node) {
    std::vector<TimeInterval> safe_intervals;
    findFreeIntervals(rt[curr_agent.start_location], safe_intervals);
    if (safe_intervals.empty()) {
        printf("Empty safe initial safe intervals!\n");
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
        // g is the actual time value to that node
        init_node->g = 0;
        // If node is not expanded, h value is the heuristic value. Otherwise,
        // we use the least value in partial interval
        init_node->h = heuristic_vec[curr_agent.id][init_node->current_point]
                                    [init_node->curr_o];
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
    allNodes_table.clear();
    useless_nodes.clear();
}

void SIPP::updateResultNodes(std::shared_ptr<Node> res,
                             Path& time_interval_path, MotionInfo& solution,
                             TimedPath& timed_path) {
    solution.clear();
    time_interval_path.clear();
    timed_path.clear();
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
    timed_path.emplace_back(curr_loc, (curr_time_min + curr_time_max) / 2.0);
    //    showSolution(res);
}

void SIPP::showSolution(std::shared_ptr<Node>& s) {
    printf("\nFor agent: %d\n", curr_agent.id);
    for (auto local_path_entry : s->bezier_solution->local_path) {
        int curr_loc = local_path_entry.location;
        double curr_time_min = local_path_entry.arrival_time;
        double curr_time_max = local_path_entry.leaving_time_tail;
        printf("Forward::The location (%d, %d, %d), the arrive time is: %f, "
               "the leave time is: %f\n",
               instance_ptr->getCoordinate(curr_loc).first,
               instance_ptr->getCoordinate(curr_loc).second, s->curr_o,
               local_path_entry.arrival_time,
               local_path_entry.leaving_time_tail);
    }
}

bool SIPP::run(int agentID, ReservationTable& rt, MotionInfo& solution,
               Path& path, double& solution_cost, TimedPath& timed_path) {
    // Init process
    Reset();
    auto debug_start_t = Time::now();
    count_called++;
    curr_agent = instance_ptr->agents[agentID];
    if (curr_agent.goal_location == curr_agent.start_location) {
        solution.clear();
        path.clear();
        solution_cost = 0.0;
        return true;
    }
    path.clear();
    double optimal_travel_time = INF;
    std::shared_ptr<Node> optimal_n = nullptr;

    std::shared_ptr<Node> init_node;
    if (!getInitNode(rt, init_node)) {
        path.clear();
        PrintNonzeroRT(rt);
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
            printf("Hit cut off time!\n");
            break;
        }
        std::shared_ptr<Node> s = open.top();
        open.pop();

        if (closed_set.find(s) == closed_set.end()) {
            closed_set.insert(s);
            count_node_closed++;
        } else if (not s->is_expanded) {
            continue;
        }

        // Found solution, exit the loop
        if (s->f > optimal_travel_time) {
            break;
        }

        // Found a goal node, might not be the optimal. Update the current
        // optimal and continue the search.
        if (s->current_point == curr_agent.goal_location and
            s->curr_o == curr_agent.end_o and s->arrival_time_max == INF) {
            optimal_travel_time = s->g;
            optimal_n = s;
            break;
        }
        // Expand the node
        else {
            // Node has been expanded before. Pop the next reachable interval
            // and do create a movement node (Line 9-10 of Algorithm 2)
            if (s->is_expanded) {
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
                                                  s->curr_o);
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
                        new_node->g = new_node->arrival_time_min;
                        new_node->h = heuristic_vec[curr_agent.id]
                                                   [new_node->current_point]
                                                   [new_node->curr_o];
                        new_node->f = new_node->g + new_node->h;
                        tpg_solution->start_t =
                            tpg_solution->local_path.front().arrival_time;
                        tpg_solution->end_t =
                            tpg_solution->local_path.back().leaving_time_tail;
                        tpg_solution->type = Action::forward;
                        new_node->bezier_solution = tpg_solution;
                        new_node->parent = s;
                        pushToOpen(new_node);
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

            // If not all the reachable intervals have been expanded,
            // we need to update the heuristic value and push it back to OPEN.
            if (not s->partial_intervals.empty()) {
                // update heuristic for s
                s->h = s->partial_intervals.top()->f;
                s->f = s->g + s->h;
                open.push(s);
                count_node_generated++;
            }
        }
    }
    auto debug_end_t = Time::now();
    time_s total_runtime_t = debug_end_t - debug_start_t;
    total_runtime_ += total_runtime_t.count();
    init_runtime += debug_init_time;
    retrieve_runtime += debug_retrieve_time;
    motion_solver_runtime += debug_bcp_time;
    expand_runtime += debug_expand_time;

    if (optimal_n == nullptr) {
        return false;
    } else {
        solution_cost = optimal_travel_time;
        updateResultNodes(optimal_n, path, solution, timed_path);
        return true;
    }
}

// return true iff we the new node is not dominated by any old node
bool SIPP::dominanceCheck(const std::shared_ptr<Node>& new_node) {
    auto ptr = allNodes_table.find(new_node);
    if (ptr == allNodes_table.end())
        return true;
    for (auto& old_node : ptr->second) {
        if ((old_node->g - new_node->g) <
            EPS) {  // the new node is dominated by the old node
            return false;
        } else  // the old node is dominated by the new node
        {       // delete the old node
            useless_nodes.insert(old_node);
            ptr->second.remove(old_node);
            count_node_generated--;  // this is because we later will increase
                                     // num_generated when we insert the new
                                     // node into lists.
            return true;
        }
    }
    return true;
}

/**
 * @brief Expand the node that has never been expand before
 *
 * @param n The node that need to be expanded
 * @param rt The reservation table
 */
void SIPP::nodeExpansion(const std::shared_ptr<Node>& n, ReservationTable& rt) {
    Neighbors neighbors;
    instance_ptr->getNeighbors(n->current_point, n->curr_o, neighbors);
    // Previous action is forward. Do rotation expansion here.
    if (n->parent == nullptr or n->prev_action == Action::forward) {
        // Insert three nodes, turn left, turn right, and turn back
        if ((n->arrival_time_min + curr_agent.rotation_cost) <
            n->arrival_time_max) {
            std::shared_ptr<Node> n_left = std::make_shared<Node>();
            std::shared_ptr<MotionNode> motion = std::make_shared<MotionNode>();
            motion->start_t = n->arrival_time_min;
            motion->end_t = n->arrival_time_min + curr_agent.rotation_cost;
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
                n->arrival_time_min + curr_agent.rotation_cost;
            n_left->arrival_time_max = n->arrival_time_max;
            n_left->g = n_left->arrival_time_min;
            n_left->h = heuristic_vec[curr_agent.id][n_left->current_point]
                                     [n_left->curr_o];
            n_left->f = n_left->g + n_left->h;
            n_left->parent = n;
            pushToOpen(n_left);
        }

        if ((n->arrival_time_min + curr_agent.rotation_cost) <
            n->arrival_time_max) {
            std::shared_ptr<Node> n_right = std::make_shared<Node>();
            std::shared_ptr<MotionNode> motion = std::make_shared<MotionNode>();
            motion->start_t = n->arrival_time_min;
            motion->end_t = n->arrival_time_min + curr_agent.rotation_cost;
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
                n->arrival_time_min + curr_agent.rotation_cost;
            n_right->arrival_time_max = n->arrival_time_max;
            n_right->g = n_right->arrival_time_min;
            n_right->h = heuristic_vec[curr_agent.id][n_right->current_point]
                                      [n_right->curr_o];
            n_right->f = n_right->g + n_right->h;
            n_right->parent = n;
            pushToOpen(n_right);
        }

        if ((n->arrival_time_min + curr_agent.turn_back_cost) <
            n->arrival_time_max) {
            std::shared_ptr<Node> n_back = std::make_shared<Node>();
            std::shared_ptr<MotionNode> motion = std::make_shared<MotionNode>();
            motion->start_t = n->arrival_time_min;
            motion->end_t = n->arrival_time_min + curr_agent.turn_back_cost;
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
                n->arrival_time_min + curr_agent.turn_back_cost;
            n_back->arrival_time_max = n->arrival_time_max;
            n_back->g = n_back->arrival_time_min;
            n_back->h = heuristic_vec[curr_agent.id][n_back->current_point]
                                     [n_back->curr_o];
            n_back->f = n_back->g + n_back->h;
            n_back->parent = n;
            pushToOpen(n_back);
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

void SIPP::getHeuristic(const std::string& heuristic_file) {
    ifstream myfile(heuristic_file.c_str());
    if (myfile.is_open()) {
        printf("success in loading heuristic!\n");

        heuristic_vec = loadVector(heuristic_file);
    } else {
        printf("Start to compute global heuristic!\n");
        size_t numCores = std::thread::hardware_concurrency();
        numCores = min(5, (int)numCores);
        for (size_t i = 0; i < instance_ptr->num_of_agents; i += numCores) {
            std::vector<std::thread> threads;
            // Pass member function and object reference
            for (size_t j = i;
                 j < min(instance_ptr->num_of_agents, (int)(i + numCores));
                 j++) {
                threads.emplace_back(&SIPP::Dijkstra, this, j);
            }
            for (auto& thread : threads) {
                thread.join();
            }
        }
    }
}

/**
 * @brief Help function, get the heuristic values
 *
 * @param start_loc The start location of the agent
 * @return Bool value determine if the search success
 */
bool SIPP::Dijkstra(size_t curr_id) {
    std::vector<std::vector<double>> curr_agent_heuristic(
        instance_ptr->map_size, std::vector<double>(NUM_ORIENT));
    std::priority_queue<std::shared_ptr<Node>,
                        std::vector<std::shared_ptr<Node>>, NodeCompare>
        dij_open;
    std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual>
        dij_close_set;
    std::shared_ptr<Node> root_node = std::make_shared<Node>();
    root_node->g = 0.0;
    root_node->f = 0.0;
    root_node->current_point = instance_ptr->agents[curr_id].goal_location;
    root_node->curr_o = instance_ptr->agents[curr_id].end_o;
    root_node->parent = nullptr;
    dij_open.push(root_node);
    size_t h_count = 0;
    while (!dij_open.empty()) {
        std::shared_ptr<Node> n = dij_open.top();
        dij_open.pop();
        auto close_item_it = dij_close_set.find(n);
        if (close_item_it != dij_close_set.end()) {
            if (close_item_it->get()->f <= n->f) {
                continue;
            } else {
                dij_close_set.erase(close_item_it);
                curr_agent_heuristic[n->current_point][n->curr_o] = n->g * 2;
                dij_close_set.insert(n);
            }
        } else {
            assert(n->current_point < curr_agent_heuristic.size());
            assert(n->curr_o < curr_agent_heuristic[0].size());
            curr_agent_heuristic[n->current_point][n->curr_o] = n->g * 2;
            h_count++;
            dij_close_set.insert(n);
        }
        // For all the neighbor location, all need to do this operation
        Neighbors neighbors;
        instance_ptr->getInverseNeighbors(n->current_point, n->curr_o,
                                          neighbors);
        if (n->parent == nullptr or n->prev_action == Action::forward) {
            // Insert two nodes, turn left and turn right
            std::shared_ptr<Node> n_left = std::make_shared<Node>();
            n_left->is_expanded = false;
            n_left->prev_action = Action::turnLeft;
            n_left->current_point = n->current_point;
            n_left->curr_o = neighbors.left.second;
            n_left->g = n->g + curr_agent.rotation_cost;
            n_left->f = n_left->g;
            n_left->parent = n;
            dij_open.push(n_left);

            std::shared_ptr<Node> n_right = std::make_shared<Node>();
            n_right->is_expanded = false;
            n_right->prev_action = Action::turnRight;
            n_right->current_point = n->current_point;
            n_right->curr_o = neighbors.right.second;
            n_right->g = n->g + curr_agent.rotation_cost;
            n_right->f = n_right->g;
            n_right->parent = n;
            dij_open.push(n_right);

            std::shared_ptr<Node> n_back = std::make_shared<Node>();
            n_back->is_expanded = false;
            n_back->prev_action = Action::turnBack;
            n_back->current_point = n->current_point;
            n_back->curr_o = neighbors.back.second;
            n_back->g = n->g + curr_agent.turn_back_cost;
            n_back->f = n_back->g;
            n_back->parent = n;
            dij_open.push(n_back);
        }

        for (int i = 0; i < neighbors.forward_locs.size(); i++) {
            std::shared_ptr<Node> n_back = std::make_shared<Node>();
            n_back->is_expanded = false;
            n_back->prev_action = Action::forward;
            n_back->current_point = neighbors.forward_locs[i];
            n_back->curr_o = n->curr_o;
            n_back->g = n->g + arrLowerBound(i);
            n_back->f = n_back->g;
            n_back->parent = n;
            dij_open.push(n_back);
        }
    }
    std::lock_guard<std::mutex> lock(mutex);
    heuristic_vec[curr_id] = curr_agent_heuristic;
    return false;
}

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
    // TODO: Split t_min into time of waiting and time of moving. Then compute
    // weighted t_min
    double m_time_min = travel_cost[min((int)prev_interval->step, 4)];
    // Compute lower bound as lb + t_min
    double lower_bound = prev_interval->t_min + m_time_min;
    std::vector<TimeInterval> interval_list;
    findFreeIntervals(rt[next_loc], interval_list);
    for (TimeInterval interval : interval_list) {
        // If lower bound is bigger than t_max of candidate interval, the
        // candidate interval is invalid If t_min of candidate interval is
        // larger than t_max of the previous interval, no transition exists for
        // such a case
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
        new_interval->f = new_interval->t_min +
                          heuristic_vec[curr_agent.id][next_loc][s->curr_o];
        new_interval->location = next_loc;
        new_interval->step = prev_interval->step + 1;
        new_interval->prev_entry = prev_interval;
        GetNextInterval(new_interval, new_interval_list, next_locs, rt, s);
        new_interval_list.push(new_interval);
    }
}

/**
 * @brief Get the potential intervals that can be obtained from moving forward
 *
 * @param s The node need to be expanded
 * @param to_locs The potential locations that can be reached from the expanded
 * node
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
    init_interval->f =
        heuristic_vec[curr_agent.id][s->current_point][s->curr_o];
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
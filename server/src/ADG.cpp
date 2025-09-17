#include "ADG.h"

ADG::ADG(int num_robots, int screen, int look_ahead_dist)
    : num_robots(num_robots), screen(screen), look_ahead_dist(look_ahead_dist) {
    //    vector<ADGNode> graph;
    // look_ahead_dist = 5;
    map<int, int> lastActionIndexByRobot;
    finished_node_idx.resize(num_robots, -1);
    enqueue_nodes_idx.resize(num_robots);
    init_locs.resize(num_robots);
}

void ADG::addMAPFPlan(const vector<vector<Action>>& plans) {
    if (graph.empty()) {
        graph.resize(num_robots);
        get_initial_plan = true;
    }
    vector<int> graph_offset(num_robots, -1);
    for (int i = 0; i < num_robots; i++) {
        graph_offset[i] = static_cast<int>(graph[i].size());
    }

    // Initialize nodes in the graph
    for (int i = 0; i < num_robots; i++) {
        for (const auto& action : plans[i]) {
            ADGNode node{action, static_cast<int>(graph[i].size()), {}, {}};
            graph[i].push_back(node);
        }
    }

    // Find the type-2 edges of the current episode
    for (int i = 0; i < num_robots; i++) {
        for (int j = 0; j < plans[i].size(); j++) {
            for (int robot_id = 0; robot_id < num_robots; robot_id++) {
                if (i == robot_id) {
                    continue;
                }
                int latest_finished_idx = finished_node_idx[robot_id];
                // Indicating no node is finished yet
                int next_node_idx = latest_finished_idx + 1;
                for (int prev_idx = next_node_idx;
                     prev_idx < graph_offset[robot_id]; prev_idx++) {
                    if (graph[robot_id][prev_idx].action.start ==
                        plans[i][j].goal) {
                        shared_ptr<Edge> tmp_edge = make_shared<Edge>(
                            robot_id, i, prev_idx, j + graph_offset[i]);
                        graph[i][j + graph_offset[i]].incomeEdges.push_back(
                            tmp_edge);
                        graph[robot_id][prev_idx].outEdges.push_back(tmp_edge);
                    }
                }
            }
        }
    }

    // Create Type 2 edges
    for (int i = 0; i < plans.size(); i++) {
        adg_stats.type1EdgeCount += static_cast<int>(plans[i].size()) - 1;
        adg_stats.totalNodes += static_cast<int>(plans[i].size());
        bool consecutive_move = false;
        for (int j = 0; j < plans[i].size(); j++) {
            // Iterate through all nodes to count type 2 edges and actions
            if (plans[i][j].type == 'M') {
                adg_stats.moveActionCount++;
                if (not consecutive_move) {
                    consecutive_move = true;
                    adg_stats.consecutiveMoveSequences++;
                }
            } else if (plans[i][j].type == 'T') {
                adg_stats.rotateActionCount++;
                consecutive_move = false;
            } else if (plans[i][j].type == 'S' or plans[i][j].type == 'P') {
                consecutive_move = false;
            } else {
                cerr << "Invalid type!\n" << endl;
                consecutive_move = false;
            }
            for (int k = i + 1; k < plans.size(); k++) {
                for (int l = 0; l < plans[k].size(); l++) {
                    bool found_conflict = false;
                    // printf("The time between: %f and %f\n", plans[i][j].time,
                    //        plans[k][l].time);
                    if (plans[i][j].start == plans[k][l].goal &&
                        plans[i][j].time <= plans[k][l].time) {
                        shared_ptr<Edge> tmp_edge = make_shared<Edge>(
                            i, k, j + graph_offset[i], l + graph_offset[k]);
                        graph[i][j + graph_offset[i]].outEdges.push_back(
                            tmp_edge);
                        graph[k][l + graph_offset[k]].incomeEdges.push_back(
                            tmp_edge);
                        found_conflict = true;
                    } else if (plans[k][l].start == plans[i][j].goal &&
                               plans[k][l].time <= plans[i][j].time) {
                        shared_ptr<Edge> tmp_edge = make_shared<Edge>(
                            k, i, l + graph_offset[k], j + graph_offset[i]);
                        graph[i][j + graph_offset[i]].incomeEdges.push_back(
                            tmp_edge);
                        graph[k][l + graph_offset[k]].outEdges.push_back(
                            tmp_edge);
                        found_conflict = true;
                    }
                    // if (plans[i][j].start == plans[k][l].goal or
                    // plans[k][l].start == plans[i][j].goal) {
                    //     assert(plans[i][j].time != plans[k][l].time);
                    // }
                }
            }
        }
    }
    if (this->screen > 0) {
        // printf("Finish building ADG graph!\n");
        spdlog::info("Finish building ADG graph!");
        if (this->screen > 1) {
            showGraph();
        }
    }

    // if (hasCycle()) {
    //     cout << "Cycle detected!" << endl;
    //     string input;
    //     getline(cin, input);
    //     exit(-1);
    // }
}

// Check if the given coordinate ends with .0. If it does, it is a added stop
// in the middle of the grids.
bool isAddStop(double x) {
    double frac = min(x - floor(x), ceil(x) - x);
    if (fabs(frac) < EPS) {
        // it ends with .0
        return false;
    }
    return true;
}

// [first commited action, last commited action + 1)
// If first commited action == last commited action + 1, no action is commited
bool ADG::fixInconsistentIncomingEdge(
    vector<pair<int, int>>& commited_actions) {
    vector<bool> commit_cut_state(num_robots, false);
    while (not all_of(commit_cut_state.begin(), commit_cut_state.end(),
                      [](bool v) { return v; })) {
        for (int agent_id = 0; agent_id < num_robots; agent_id++) {
            if (not commit_cut_state[agent_id]) {
                if (commited_actions[agent_id].first !=
                    commited_actions[agent_id].second) {
                    double tmp_x =
                        graph[agent_id][commited_actions[agent_id].second - 1]
                            .action.goal.first;
                    double tmp_y =
                        graph[agent_id][commited_actions[agent_id].second - 1]
                            .action.goal.second;
                    if (isAddStop(tmp_x) or isAddStop(tmp_y)) {
                        assert(commited_actions[agent_id].second <
                               graph[agent_id].size());
                        commited_actions[agent_id].second++;
                    }
                }
                for (int tmp_action = commited_actions[agent_id].first;
                     tmp_action < commited_actions[agent_id].second;
                     tmp_action++) {
                    for (auto& tmp_in_edge :
                         graph[agent_id][tmp_action].incomeEdges) {
                        if (tmp_in_edge->from_node_id >=
                            commited_actions[tmp_in_edge->from_agent_id]
                                .second) {
                            commited_actions[tmp_in_edge->from_agent_id]
                                .second = tmp_in_edge->from_node_id + 1;
                            commit_cut_state[tmp_in_edge->from_agent_id] =
                                false;
                        }
                    }
                }
            } else {
                ;
            }
            commit_cut_state[agent_id] = true;
        }
    }
    return true;
}

vector<robotState> ADG::computeCommitCut() {
    if (not initialized) {
        // cout << "computeCommitCut::Not initialized!" << endl;
        spdlog::warn("ADG::computeCommitCut: Not initialized!");
        return {};
    }
    curr_commit.clear();
    // If it is never initialized
    if (graph.size() == 0) {
        for (int agent_id = 0; agent_id < num_robots; agent_id++) {
            curr_commit.emplace_back(init_locs[agent_id].position, 0);
        }
        return curr_commit;
    }

    vector<pair<int, int>> commited_actions(num_robots);
    for (int agent_id = 0; agent_id < num_robots; agent_id++) {
        // set all enqueue actions as commited
        commited_actions[agent_id] = make_pair(finished_node_idx[agent_id] + 1,
                                               finished_node_idx[agent_id] + 1);

        // We first find the first node idx that is not enqueued. All enqueued
        // actions must be commited.
        if (not enqueue_nodes_idx[agent_id].empty()) {
            assert(enqueue_nodes_idx[agent_id].front() ==
                   commited_actions[agent_id].first);
            commited_actions[agent_id].second =
                enqueue_nodes_idx[agent_id].back() + 1;
        }
        // If number of enqueued action is smaller than the pre-defined
        // look-ahead-dist, we then find actions that are staged
        int num_commit_actions = commited_actions[agent_id].second -
                                 commited_actions[agent_id].first;
        if (num_commit_actions < this->look_ahead_dist) {
            commited_actions[agent_id].second =
                min((int)graph[agent_id].size(),
                    commited_actions[agent_id].first + this->look_ahead_dist);
        }
    }
    fixInconsistentIncomingEdge(commited_actions);
#ifdef DEBUG
    cout << "commited actions after fix: " << endl;
    for (int agent_id = 0; agent_id < num_robots; agent_id++) {
        cout << "Agent " << agent_id << ": " << commited_actions[agent_id].first
             << " -> " << commited_actions[agent_id].second << endl;
    }
#endif
    unordered_map<pair<double, double>, vector<int>, pair_hash>
        duplicate_starts;
    for (int agent_id = 0; agent_id < num_robots; agent_id++) {
        if (this->screen > 1) {
            spdlog::info("Agent {}: {} -> {}", agent_id,
                         commited_actions[agent_id].first,
                         commited_actions[agent_id].second);
            // cout << "Agent " << agent_id << ": "
            //           << commited_actions[agent_id].first << " -> "
            //           << commited_actions[agent_id].second << endl;
        }

        pair<double, double> tmp_loc;
        if (graph[agent_id].empty()) {
            tmp_loc = init_locs[agent_id].position;
        } else {
            tmp_loc = graph[agent_id][commited_actions[agent_id].second - 1]
                          .action.goal;
        }
        if (duplicate_starts.find(tmp_loc) != duplicate_starts.end()) {
            spdlog::warn(
                "Duplicate start location found! Agent {}. Duplicate start "
                "location: {}, {}",
                agent_id, tmp_loc.first, tmp_loc.second);
            // cerr << "Duplicate start location found! Agent " << agent_id
            //           << ". Duplicate start location: " << tmp_loc.first <<
            //           ", "
            //           << tmp_loc.second << endl;
            graph[agent_id].back().showNode();
            string skip_info;
            // cerr << "Duplicate agent: " << endl;
            spdlog::warn("Duplicate agent: ");
            for (int dup_agent_id : duplicate_starts[tmp_loc]) {
                // cerr << dup_agent_id << ": ";
                spdlog::warn("{}: ", dup_agent_id);
                graph[dup_agent_id].back().showNode();
            }
            // cerr << "Continue? y/n" << endl;
            // cin >> skip_info;
        }
        duplicate_starts[tmp_loc].push_back(agent_id);
    }

    // pop out unused actions
    for (int agent_id = 0; agent_id < num_robots; agent_id++) {
        assert(commited_actions[agent_id].second <= graph[agent_id].size());
        assert(commited_actions[agent_id].first <=
               commited_actions[agent_id].second);
        // when remove nodes not commited, also remove the type-2 edges
#ifdef DEBUG
        cout << "Clean commited actions for agent " << agent_id
             << ", total actions: " << graph[agent_id].size() << endl;
#endif
        if (graph[agent_id].empty()) {
            curr_commit.emplace_back(init_locs[agent_id].position, 0);
            continue;
        }
        for (int node_idx = commited_actions[agent_id].second;
             node_idx < graph[agent_id].size(); node_idx++) {
            // we can ignore outgoing edges
            for (auto& tmp_in_edge : graph[agent_id][node_idx].incomeEdges) {
                // auto& vec =
                // graph[tmp_in_edge->from_agent_id][tmp_in_edge->from_node_id].outEdges;
                // vec.erase(remove(vec.begin(), vec.end(), tmp_in_edge),
                // vec.end());
                tmp_in_edge->valid = false;
            }
        }
        // cout << "Clean commited actions for agent " << agent_id << ".
        // total graph size: "
        //     << graph[agent_id].size() << ",remove start from: " <<
        //     commited_actions[agent_id].second << endl;
        graph[agent_id].erase(
            graph[agent_id].begin() + commited_actions[agent_id].second,
            graph[agent_id].end());
        // cout << "Remove rest of actions for agent " << agent_id <<
        // endl;

        ADGNode last_node = graph[agent_id].back();
        auto action = last_node.action;
        // commitCut[agent_id] = last_node.action.goal;
        curr_commit.emplace_back(
            last_node.action.goal,
            static_cast<int>(last_node.action.orientation));
        // cout << "Loop end for agent " << agent_id << ": " << endl;
        // cout << "curr size of commited is: " << curr_commit.size() << ":
        // graph size: " << graph.size() << endl; cout << "        {"
        // << action.robot_id << ", " << action.time << ", "
        //   << fixed << setprecision(1) << action.orientation << ",
        //   '"
        //   << action.type << "', {" << action.start.first << ", " <<
        //   action.start.second << "}, {"
        //   << action.goal.first << ", " << action.goal.second << "}, " <<
        //   action.nodeID  << "}," << endl;
    }
#ifdef DEBUG
    cout << "Find commit Cut " << endl;
#endif
    // printProgress();
    if (this->screen > 0) {
        spdlog::info("Commit cut number: {}, total number of robots: {}",
                     curr_commit.size(), num_robots);
        // cout << "Commit cut number: " << curr_commit.size()
        //           << ", total number of robots: " << num_robots << endl;
    }

    return curr_commit;
}

void printEdge() {
    // printf("Found type-2 edge from agent %d Node %d (%f, %f) to (%f, %f) ->
    // agent %d Node %d (%f, %f) to (%f, %f).\n",
    //     i, j, plans[i][j].start.first, plans[i][j].start.second,
    //     plans[i][j].goal.first, plans[i][j].goal.second, k, l,
    //     plans[k][l].start.first, plans[k][l].start.second,
    //     plans[k][l].goal.first, plans[k][l].goal.second);

    // cout << "Type 2 edge detected: From Node " << plans[i][j].nodeID <<
    // " to Node " << plans[k][l].nodeID << endl; cout << "i, j, k,
    // l:" << i << ", " << j << ", " << k << ", " << l << endl; cout
    // << "plan 1 start: " << plans[i][j].start.first << " , " <<
    // plans[i][j].start.second << endl; cout << "plan 1 end: " <<
    // plans[i][j].goal.first << " , " << plans[i][j].goal.second << endl;
    // cout << "plan 2 start: " << plans[k][l].start.first << " , " <<
    // plans[k][l].start.second << endl; cout << "plan 2 end: " <<
    // plans[k][l].goal.first << " , " << plans[k][l].goal.second << endl;
}

// pair<map<int, string>, map<string, int>>
// ADG::createRobotIDToStartIndexMaps() {
//     // map<int, string> robotIDToStartIndex;
//     // map<string, int> startIndexToRobotID;

//     for (int robot_id = 0; robot_id < num_robots; robot_id++) {
//         auto start = graph[robot_id][0].action.start;
//         ostringstream oss;
//         oss << static_cast<int>(start.first) << "_"
//             << static_cast<int>(start.second);
//         string startStr = oss.str();

//         robotIDToStartIndex[robot_id] = startStr;
//         startIndexToRobotID[startStr] = robot_id;
//     }

//     return {robotIDToStartIndex, startIndexToRobotID};
// }

bool ADG::createRobotIDToStartIndexMaps(string& robot_id_str,
                                        tuple<int, int> init_loc) {
    // map<int, string> robotIDToStartIndex;
    // map<string, int> startIndexToRobotID;
    if (startIndexToRobotID.find(robot_id_str) != startIndexToRobotID.end()) {
        return false;
    }

    int robot_id = stoi(robot_id_str);
    robotIDToStartIndex[robot_id] = robot_id_str;
    startIndexToRobotID[robot_id_str] = robot_id;

    // init_locs.emplace_back(get<0>(init_loc), get<1>(init_loc));
    init_locs[robot_id] = robotState(static_cast<int>(get<0>(init_loc)),
                                     static_cast<int>(get<1>(init_loc)));
    this->n_robot_init++;

    if (this->n_robot_init == num_robots) {
        initialized = true;
        robot_states = init_locs;
        return true;
    }
    return false;

    // int robot_id = static_cast<int>(init_locs.size());
    // robotIDToStartIndex[robot_id] = robot_id_str;
    // startIndexToRobotID[robot_id_str] = robot_id;

    // size_t pos = robot_id_str.find('_');
    // if (pos == string::npos) {
    //     cerr << "Invalid robot locations!" << endl;
    //     exit(-1);
    // }

    // int x = stoi(robot_id_str.substr(0, pos));
    // int y = stoi(robot_id_str.substr(pos + 1));
    // init_locs.emplace_back(x, y);

    // if (init_locs.size() == num_robots) {
    //     initialized = true;
    //     robot_states = init_locs;
    //     return true;
    // }
    // return false;
}

void inline updateADGNode(ADGNode& tmp_node) {
    bool valid = false;
    for (auto& tmp_in_edge : tmp_node.incomeEdges) {
        if (tmp_in_edge->valid) {
            valid = true;
            break;
        }
    }
    tmp_node.has_valid_in_edge = valid;
}

// Function to find the first node associated with a given robot_id
bool ADG::getAvailableNodes(int robot_id, vector<int>& available_nodes) {
    auto& curr_agent_plan = graph[robot_id];
    int latest_finished_idx = finished_node_idx[robot_id];
    // Indicating no node is finished yet
    int next_node_idx = latest_finished_idx + 1;
    int num_additional_nodes = MAX_ENQUE - enqueue_nodes_idx[robot_id].size();
    for (int i = next_node_idx; i < curr_agent_plan.size(); i++) {
        if (curr_agent_plan[i].has_valid_in_edge) {
            updateADGNode(curr_agent_plan[i]);
        }

        // or available_nodes.size() >= num_additional_nodes
        if (curr_agent_plan[i].has_valid_in_edge or
            available_nodes.size() >= num_additional_nodes) {
            break;
        }

        if (enqueue_nodes_idx[robot_id].empty() or
            enqueue_nodes_idx[robot_id].back() < i) {
            available_nodes.push_back(i);
        }
    }
    return true;
}

bool ADG::updateFinishedNode(int robot_id, int node_id) {
    int latest_finished_idx = finished_node_idx[robot_id];
    if (node_id <= latest_finished_idx) {
        // cerr << "Reconfirming nodes!" << endl;
        spdlog::warn("ADG::updateFinishedNode: Reconfirming nodes!");
        return true;
    } else {
        if (not enqueue_nodes_idx[robot_id].empty() and
            node_id > enqueue_nodes_idx[robot_id].back()) {
            // cerr << "Confirm for nodes never enqueue!" << endl;
            spdlog::warn("ADG::updateFinishedNode: Confirm for nodes never "
                         "enqueue!");
            return false;
        } else {
            // remove type-2 edges
            for (int tmp_idx = latest_finished_idx + 1; tmp_idx <= node_id;
                 tmp_idx++) {
                for (auto& tmp_edge : graph[robot_id][tmp_idx].incomeEdges) {
                    assert(not tmp_edge->valid);
                }

                for (auto& tmp_edge : graph[robot_id][tmp_idx].outEdges) {
                    tmp_edge->valid = false;
                }
            }
            finished_node_idx[robot_id] = node_id;
            while (not enqueue_nodes_idx[robot_id].empty()) {
                if (enqueue_nodes_idx[robot_id].front() <= node_id) {
                    enqueue_nodes_idx[robot_id].pop_front();
                } else {
                    break;
                }
            }
            robot_states[robot_id].position =
                graph[robot_id][node_id].action.goal;
            robot_states[robot_id].orient =
                graph[robot_id][node_id].action.orientation;
            return true;
        }
    }
}

bool ADG::isTaskNode(int robot_id, int node_id) {
    if (not graph.empty()) {
        if (graph[robot_id][node_id].action.type == 'S' or
            graph[robot_id][node_id].action.type == 'P') {
            return true;
        }
    }
    return false;
}

// void ADG::setEnqueueNodes(int robot_id, vector<int>& enqueue_nodes) {
//     auto& curr_enqueue = enqueue_nodes_idx[robot_id];
//     if (curr_enqueue.empty()) {
//         curr_enqueue.insert(curr_enqueue.end(), enqueue_nodes.begin(),
//         enqueue_nodes.end());
//     } else {
//         int i = 0;
//         for (i = 0; i < enqueue_nodes.size(); i++) {
//             if (enqueue_nodes[i] > curr_enqueue.back()) {
//                 break;
//             }
//         }
//         curr_enqueue.insert(curr_enqueue.end(), enqueue_nodes.begin()+i,
//         enqueue_nodes.end());
//     }
// }

void ADG::findConstraining(int robot_id) {
    auto& curr_agent_plan = graph[robot_id];
    int latest_finished_idx = finished_node_idx[robot_id];
    // Indicating no node is finished yet
    int next_node_idx = latest_finished_idx + 1;
    for (int i = next_node_idx; i < curr_agent_plan.size(); i++) {
        if (curr_agent_plan[i].has_valid_in_edge) {
            updateADGNode(curr_agent_plan[i]);
        }

        if (curr_agent_plan[i].has_valid_in_edge) {
            cout << "Constraining idx: " << i << ";";
            for (auto tmp_edge : curr_agent_plan[i].incomeEdges) {
                if (tmp_edge->valid) {
                    cout << " Constraint Agent " << tmp_edge->from_agent_id
                         << " at node " << tmp_edge->from_node_id << ";";
                }
            }
            break;
        }
    }
}

void ADG::printActions(
    const vector<tuple<string, int, double, string, pair<double, double>,
                       pair<double, double>>>& actions) {
    for (const auto& action : actions) {
        string robot_id = get<0>(action);
        int time = get<1>(action);
        double orientation = get<2>(action);
        string type = get<3>(action);
        auto start = get<4>(action);
        auto goal = get<5>(action);

        cout << "Robot ID: " << robot_id << ", nodeID: " << time
             << ", Orientation: " << orientation << ", Type: " << type
             << ", Start: (" << start.first << ", " << start.second << ")"
             << ", Goal: (" << goal.first << ", " << goal.second << ")" << endl;
    }
}

SIM_PLAN ADG::getPlan(int agent_id) {
    SIM_PLAN sim_plan;
    vector<int> enque_acts;
    getAvailableNodes(agent_id, enque_acts);
    // TODO@jingtian: rethink logic here if we have data loss
    for (int enque_id : enque_acts) {
        const Action& action = graph[agent_id][enque_id].action;
        // pair<int, int> intStart = action.start;
        // pair<int, int> intEnd = action.goal;
        // pair<double, double> doubleStart =
        // {static_cast<double>(intStart.first),
        // static_cast<double>(intStart.second)}; pair<double, double>
        // doubleEnd = {static_cast<double>(intEnd.first),
        // static_cast<double>(intEnd.second)};
        int task_id;
        if (action.task_ptr == nullptr) {
            task_id = -1;
        } else {
            task_id = action.task_ptr->id;
        }
        sim_plan.emplace_back(robotIDToStartIndex[action.robot_id], enque_id,
                              action.orientation, string(1, action.type),
                              action.start, action.goal, task_id);
        enqueue_nodes_idx[agent_id].push_back(enque_id);
    }
    // cout << "For agent " << agent_id << ", enqueue size is: " <<
    // enqueue_nodes_idx[agent_id].size() << endl;
    return sim_plan;
}

set<int> ADG::updateFinishedTasks() {
    if (curr_commit.empty() or graph.empty()) {
        return {};
    }
    // finish_tasks.clear();
    // finish_tasks.resize(num_robots);
    set<int> new_finished_tasks;
    for (int agent_id = 0; agent_id < num_robots; agent_id++) {
        for (int j = graph[agent_id].size() - 1; j >= 0; j--) {
            // Current ADGNode has a non-negative task_id, then we finish a
            // task.
            // Note: this function is invoked after ComputeCommitCut, which
            // removed actions that are to be replaced by future plans.
            int curr_task = graph[agent_id][j].action.task_id;
            // cout << "Agent " << agent_id << ", Node "
            //      << graph[agent_id][j].node_id
            //      << ", Action: " << graph[agent_id][j].action.type << " at ("
            //      << graph[agent_id][j].action.start.first << ", "
            //      << graph[agent_id][j].action.start.second << ") -> ("
            //      << graph[agent_id][j].action.goal.first << ", "
            //      << graph[agent_id][j].action.goal.second << ")"
            //      << ", Task ID: " << curr_task << endl;
            if (this->finished_tasks_.find(curr_task) !=
                this->finished_tasks_.end()) {
                break;
            } else if (curr_task >= 0 &&
                       this->finished_tasks_.find(curr_task) ==
                           this->finished_tasks_.end()) {
                this->finished_tasks_.insert(curr_task);
                new_finished_tasks.insert(curr_task);

                // Only count the task that are not backup tasks to throughput
                n_finished_tasks++;
                if (this->backup_tasks.find(curr_task) !=
                    this->backup_tasks.end()) {
                    n_finished_backup_tasks++;
                }
            }
        }
    }
    return new_finished_tasks;
}

json ADG::getADGStats() {
    json result;
    if (graph.empty()) {
        return result;
    }
    double total_rotation = 0.0;
    double total_move = 0.0;

    // Path of the robots. Each element is a list of (x, y, orientation,
    // task_id)
    vector<vector<tuple<double, double, double, int>>> robot_paths(num_robots);
    for (int agent_id = 0; agent_id < num_robots; agent_id++) {
        // 0 indicates the initial orientation of North
        robot_paths[agent_id].push_back({init_locs[agent_id].position.second,
                                         init_locs[agent_id].position.first,
                                         0.0, -1});
    }
    for (int agent_id = 0; agent_id < num_robots; agent_id++) {
        for (int j = 0; j < finished_node_idx[agent_id]; j++) {
            // Count the number of different actions
            if (graph[agent_id][j].action.type == 'T') {
                if (j > 0) {
                    int degree;
                    int abs_ = abs(graph[agent_id][j].action.orientation -
                                   graph[agent_id][j - 1].action.orientation);
                    if (graph[agent_id][j].action.orientation ==
                        graph[agent_id][j - 1].action.orientation)
                        degree = 0;
                    else if (abs_ == 1 || abs_ == 3)
                        degree = 1;
                    else {
                        spdlog::warn(
                            "ADG::getADGStats: Unexpected orientation change "
                            "from {} to {}",
                            graph[agent_id][j - 1].action.orientation,
                            graph[agent_id][j].action.orientation);
                        degree = 2;
                    }
                    total_rotation += degree;
                } else
                    total_rotation++;
            } else if (graph[agent_id][j].action.type == 'M') {
                total_move++;
            }

            // Record the agents path
            auto action = graph[agent_id][j].action;
            robot_paths[agent_id].push_back(
                {action.goal.second, action.goal.first, action.orientation,
                 action.task_id});
        }
    }
    int total_actions = total_rotation + total_move;
    result["mean_avg_rotation"] = total_rotation / num_robots;
    result["mean_avg_move"] = total_move / num_robots;
    result["avg_total_actions"] = total_actions / num_robots;
    result["robot_paths"] = robot_paths;
    return result;
}

void ADG::showGraph() {
    for (size_t i = 0; i < numRobots(); i++) {
        // if (graph[i].size() - finished_node_idx[i] < 5) {
        printf("Graph of agent: %lu\n", i);
        // Show the unfinished graph of the each agent
        for (int j = finished_node_idx[i] + 1; j < graph[i].size(); j++) {
            auto& action_node = graph[i][j];
            auto action = action_node.action;
            cout << "        {" << action.robot_id << ", " << action.time
                 << ", " << fixed << setprecision(1) << action.orientation
                 << ", '" << action.type << "', " << action.task_id << " , {"
                 << action.start.first << ", " << action.start.second << "}, {"
                 << action.goal.first << ", " << action.goal.second << "}, "
                 << action.nodeID << "}. Node idx: " << action_node.node_id
                 << ", idx in graph is:" << j << ", Out edges: ";
            for (auto& tmp_out_edge : action_node.outEdges) {
                cout << "agent " << tmp_out_edge->to_agent_id << ", node "
                     << tmp_out_edge->to_node_id << ", status "
                     << tmp_out_edge->valid << ";";
            }
            cout << "\tIn edges: ";
            for (auto& tmp_in_edge : action_node.incomeEdges) {
                cout << "agent " << tmp_in_edge->from_agent_id << ", node "
                     << tmp_in_edge->from_node_id << ", status "
                     << tmp_in_edge->valid << ";";
            }
            cout << endl;
            // j++;
        }
        printf("\n");
        // }
    }
}

bool ADG::dfs(int agent_id, int node_id,
              unordered_map<int, unordered_set<int>>& visited,
              unordered_map<int, unordered_set<int>>& recStack,
              const vector<vector<ADGNode>>& graph,
              unordered_map<loopNode, loopNode, NodeHash>& parent,
              vector<loopNode>& cycle_path) {
    visited[agent_id].insert(node_id);
    recStack[agent_id].insert(node_id);

    const ADGNode& node = graph[agent_id][node_id];
    for (const auto& edge : node.outEdges) {
        if (!edge->valid)
            continue;
        int next_agent = edge->to_agent_id;
        int next_node = edge->to_node_id;

        if (visited[next_agent].count(next_node) == 0) {
            parent[{next_agent, next_node}] = {agent_id, node_id};
            if (dfs(next_agent, next_node, visited, recStack, graph, parent,
                    cycle_path)) {
                return true;
            }
        } else if (recStack[next_agent].count(next_node)) {
            // Cycle detected!
            // Reconstruct cycle path
            loopNode current = {agent_id, node_id};
            cycle_path.push_back({next_agent, next_node});
            while (current != loopNode{next_agent, next_node}) {
                cycle_path.push_back(current);
                current = parent[current];
            }
            cycle_path.push_back({next_agent, next_node});  // close the loop
            std::reverse(cycle_path.begin(), cycle_path.end());
            return true;
        }
    }

    recStack[agent_id].erase(node_id);
    return false;
}

bool ADG::hasCycle() {
    unordered_map<int, unordered_set<int>> visited;
    unordered_map<int, unordered_set<int>> recStack;
    unordered_map<loopNode, loopNode, NodeHash> parent;
    vector<loopNode> cycle_path;

    for (int agent_id = 0; agent_id < graph.size(); ++agent_id) {
        for (const auto& node : graph[agent_id]) {
            if (visited[agent_id].count(node.node_id) == 0) {
                if (dfs(agent_id, node.node_id, visited, recStack, graph,
                        parent, cycle_path)) {
                    // Print the cycle path
                    cout << "Cycle detected:\n";
                    for (const auto& [aid, nid] : cycle_path) {
                        cout << "(Agent " << aid << ", Node " << nid << ") -> ";
                    }
                    cout << "(back to start)\n";
                    return true;
                }
            }
        }
    }

    cout << "No cycle detected.\n";
    return false;
}

void ADG::printProgress() {
    //        exit(0);
    for (int agent_id = 0; agent_id < num_robots; agent_id++) {
        cout << "Agent " << agent_id
             << ", ID: " << robotIDToStartIndex[agent_id] << " with plan size "
             << graph[agent_id].size() << ": ";
        findConstraining(agent_id);
        for (int i = 0; i <= finished_node_idx[agent_id]; i++) {
            cout << "#";
        }
        for (auto elem : enqueue_nodes_idx[agent_id]) {
            cout << '0';
        }
        int unstart;
        if (enqueue_nodes_idx[agent_id].empty()) {
            unstart = finished_node_idx[agent_id];
        } else {
            unstart = enqueue_nodes_idx[agent_id].back();
        }
        for (int i = unstart + 1; i < graph[agent_id].size(); i++) {
            cout << "*";
        }
        cout << endl;
    }
    // cerr << "Robot ID of 13 is: " << robotIDToStartIndex[13] <<
    // endl;
    cout << endl;
}

int ADG::getNumUnfinishedActions(int agent_id) {
    // ADG Graph is uninitialized
    if (graph.empty() || graph[agent_id].empty()) {
        return 0;
    }

    // ADG graph is initialized, but no actions are finished yet
    if (finished_node_idx[agent_id] < 0) {
        return graph[agent_id].size();
    }

    // Some actions are finished, return the number of unfinished actions
    // int n_unfinished_actions = 0;
    // for (int i = graph[agent_id].size() - 1; i >= 0; --i) {
    //     if (i == finished_node_idx[agent_id])
    //         break;
    //     n_unfinished_actions++;
    // }
    int n_unfinished_actions =
        graph[agent_id].size() - finished_node_idx[agent_id] - 1;
    // The actions here considers the actions that happen in the middle of the
    // grids, so we divide by 2
    return n_unfinished_actions / 2;
}
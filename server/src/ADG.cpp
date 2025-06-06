#include "ADG.h"

#include <cmath>

ADG::ADG(int num_robots) : num_robots(num_robots) {
    //    std::vector<ADGNode> graph;
    look_ahead_dist = 5;
    std::map<int, int> lastActionIndexByRobot;
    finished_node_idx.resize(num_robots, -1);
    enqueue_nodes_idx.resize(num_robots);
}

void ADG::addMAPFPlan(const std::vector<std::vector<Action>>& plans) {
    if (graph.empty()) {
        graph.resize(num_robots);
        get_initial_plan = true;
    }
    std::vector<int> graph_offset(num_robots, -1);
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
                        std::shared_ptr<Edge> tmp_edge = std::make_shared<Edge>(
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
                std::cerr << "Invalid type!\n" << std::endl;
                consecutive_move = false;
            }
            for (int k = i + 1; k < plans.size(); k++) {
                for (int l = 0; l < plans[k].size(); l++) {
                    bool found_conflict = false;
                    //                    printf("The time between: %f and
                    //                    %f\n", plans[i][j].time,
                    //                    plans[k][l].time);
                    if (plans[i][j].start == plans[k][l].goal &&
                        plans[i][j].time <= plans[k][l].time) {
                        std::shared_ptr<Edge> tmp_edge = std::make_shared<Edge>(
                            i, k, j + graph_offset[i], l + graph_offset[k]);
                        graph[i][j + graph_offset[i]].outEdges.push_back(
                            tmp_edge);
                        graph[k][l + graph_offset[k]].incomeEdges.push_back(
                            tmp_edge);
                        found_conflict = true;
                    } else if (plans[k][l].start == plans[i][j].goal &&
                               plans[k][l].time <= plans[i][j].time) {
                        std::shared_ptr<Edge> tmp_edge = std::make_shared<Edge>(
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
    printf("Finish building graph!\n");
    // if (hasCycle()) {
    //     std::cerr << "Cycle detected!" << std::endl;
    //     std::string input;
    //     std::getline(std::cin, input);
    //     // exit(-1);
    // }
}

bool isAddStop(double x) {
    double frac = std::min(x - std::floor(x), std::ceil(x) - x);
    if (std::fabs(frac) < EPS) {
        // it ends with .0
        return false;
    }
    return true;
}

// [first commited action, last commited action + 1)
// If first commited action == last commited action + 1, no action is commited
bool ADG::fixInconsistentIncomingEdge(
    std::vector<std::pair<int, int>>& commited_actions) {
    std::vector<bool> commit_cut_state(num_robots, false);
    while (not std::all_of(commit_cut_state.begin(), commit_cut_state.end(),
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

std::vector<robotState> ADG::computeCommitCut(int num_enqueue_node) {
    if (not initialized) {
        std::cout << "computeCommitCut::Not initialized!" << std::endl;
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

    std::vector<std::pair<int, int>> commited_actions(num_robots);
    for (int agent_id = 0; agent_id < num_robots; agent_id++) {
        // set all enqueue actions as commited
        commited_actions[agent_id] = std::make_pair(
            finished_node_idx[agent_id] + 1, finished_node_idx[agent_id] + 1);
        if (not enqueue_nodes_idx[agent_id].empty()) {
            assert(enqueue_nodes_idx[agent_id].front() ==
                   commited_actions[agent_id].first);
            commited_actions[agent_id].second =
                enqueue_nodes_idx[agent_id].back() + 1;
        }
        // find actions that are staged
        int num_commit_actions = commited_actions[agent_id].second -
                                 commited_actions[agent_id].first;
        if (num_commit_actions < num_enqueue_node) {
            commited_actions[agent_id].second =
                std::min((int)graph[agent_id].size(),
                         commited_actions[agent_id].first + num_enqueue_node);
        }
    }
    fixInconsistentIncomingEdge(commited_actions);
#ifdef DEBUG
    std::cout << "commited actions after fix: " << std::endl;
    for (int agent_id = 0; agent_id < num_robots; agent_id++) {
        std::cout << "Agent " << agent_id << ": "
                  << commited_actions[agent_id].first << " -> "
                  << commited_actions[agent_id].second << std::endl;
    }
#endif
    std::unordered_map<std::pair<double, double>, std::vector<int>, pair_hash>
        duplicate_starts;
    for (int agent_id = 0; agent_id < num_robots; agent_id++) {
        std::cout << "Agent " << agent_id << ": "
                  << commited_actions[agent_id].first << " -> "
                  << commited_actions[agent_id].second << std::endl;
        std::pair<double, double> tmp_loc;
        if (graph[agent_id].empty()) {
            tmp_loc = init_locs[agent_id].position;
        } else {
            tmp_loc = graph[agent_id][commited_actions[agent_id].second - 1]
                          .action.goal;
        }
        if (duplicate_starts.find(tmp_loc) != duplicate_starts.end()) {
            std::cerr << "Duplicate start location found! Agent " << agent_id
                      << ". Duplicate start location: " << tmp_loc.first << ", "
                      << tmp_loc.second << std::endl;
            graph[agent_id].back().showNode();
            string skip_info;
            std::cout << "Duplicate agent: " << std::endl;
            for (int dup_agent_id : duplicate_starts[tmp_loc]) {
                std::cout << dup_agent_id << ": ";
                graph[dup_agent_id].back().showNode();
            }
            std::cerr << "Continue? y/n" << std::endl;
            std::cin >> skip_info;
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
        std::cout << "Clean commited actions for agent " << agent_id
                  << ", total actions: " << graph[agent_id].size() << std::endl;
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
                // vec.erase(std::remove(vec.begin(), vec.end(), tmp_in_edge),
                // vec.end());
                tmp_in_edge->valid = false;
            }
        }
        // std::cout << "Clean commited actions for agent " << agent_id << ".
        // total graph size: "
        //     << graph[agent_id].size() << ",remove start from: " <<
        //     commited_actions[agent_id].second << std::endl;
        graph[agent_id].erase(
            graph[agent_id].begin() + commited_actions[agent_id].second,
            graph[agent_id].end());
        // std::cout << "Remove rest of actions for agent " << agent_id <<
        // std::endl;

        ADGNode last_node = graph[agent_id].back();
        auto action = last_node.action;
        // commitCut[agent_id] = last_node.action.goal;
        curr_commit.emplace_back(
            last_node.action.goal,
            static_cast<int>(last_node.action.orientation));
        // std::cout << "Loop end for agent " << agent_id << ": " << std::endl;
        // std::cout << "curr size of commited is: " << curr_commit.size() << ":
        // graph size: " << graph.size() << std::endl; std::cout << "        {"
        // << action.robot_id << ", " << action.time << ", "
        //   << std::fixed << std::setprecision(1) << action.orientation << ",
        //   '"
        //   << action.type << "', {" << action.start.first << ", " <<
        //   action.start.second << "}, {"
        //   << action.goal.first << ", " << action.goal.second << "}, " <<
        //   action.nodeID  << "}," << std::endl;
    }
#ifdef DEBUG
    std::cout << "Find commit Cut " << std::endl;
#endif
    // printProgress();
    return curr_commit;
}

void printEdge() {
    // printf("Found type-2 edge from agent %d Node %d (%f, %f) to (%f, %f) ->
    // agent %d Node %d (%f, %f) to (%f, %f).\n",
    //     i, j, plans[i][j].start.first, plans[i][j].start.second,
    //     plans[i][j].goal.first, plans[i][j].goal.second, k, l,
    //     plans[k][l].start.first, plans[k][l].start.second,
    //     plans[k][l].goal.first, plans[k][l].goal.second);

    // std::cout << "Type 2 edge detected: From Node " << plans[i][j].nodeID <<
    // " to Node " << plans[k][l].nodeID << std::endl; std::cout << "i, j, k,
    // l:" << i << ", " << j << ", " << k << ", " << l << std::endl; std::cout
    // << "plan 1 start: " << plans[i][j].start.first << " , " <<
    // plans[i][j].start.second << std::endl; std::cout << "plan 1 end: " <<
    // plans[i][j].goal.first << " , " << plans[i][j].goal.second << std::endl;
    // std::cout << "plan 2 start: " << plans[k][l].start.first << " , " <<
    // plans[k][l].start.second << std::endl; std::cout << "plan 2 end: " <<
    // plans[k][l].goal.first << " , " << plans[k][l].goal.second << std::endl;
}

std::pair<std::map<int, std::string>, std::map<std::string, int>>
ADG::createRobotIDToStartIndexMaps() {
    // std::map<int, std::string> robotIDToStartIndex;
    // std::map<std::string, int> startIndexToRobotID;

    for (int robot_id = 0; robot_id < num_robots; robot_id++) {
        auto start = graph[robot_id][0].action.start;
        std::ostringstream oss;
        oss << static_cast<int>(start.first) << "_"
            << static_cast<int>(start.second);
        std::string startStr = oss.str();

        robotIDToStartIndex[robot_id] = startStr;
        startIndexToRobotID[startStr] = robot_id;
    }

    return {robotIDToStartIndex, startIndexToRobotID};
}

bool ADG::createRobotIDToStartIndexMaps(std::string& robot_id_str) {
    // std::map<int, std::string> robotIDToStartIndex;
    // std::map<std::string, int> startIndexToRobotID;
    if (startIndexToRobotID.find(robot_id_str) != startIndexToRobotID.end()) {
        return false;
    }
    int robot_id = static_cast<int>(init_locs.size());
    robotIDToStartIndex[robot_id] = robot_id_str;
    startIndexToRobotID[robot_id_str] = robot_id;

    size_t pos = robot_id_str.find('_');
    if (pos == std::string::npos) {
        std::cerr << "Invalid robot locations!" << std::endl;
        exit(-1);
    }

    int x = std::stoi(robot_id_str.substr(0, pos));
    int y = std::stoi(robot_id_str.substr(pos + 1));
    init_locs.emplace_back(x, y);

    if (init_locs.size() == num_robots) {
        initialized = true;
        robot_states = init_locs;
        return true;
    }
    return false;
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
bool ADG::getAvailableNodes(int robot_id, std::vector<int>& available_nodes) {
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
        std::cerr << "Reconfirming nodes!" << std::endl;
        return true;
    } else {
        if (not enqueue_nodes_idx[robot_id].empty() and
            node_id > enqueue_nodes_idx[robot_id].back()) {
            std::cerr << "Confirm for nodes never enqueue!" << std::endl;
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

// void ADG::setEnqueueNodes(int robot_id, std::vector<int>& enqueue_nodes) {
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
            std::cout << "Constraining idx: " << i << ";";
            for (auto tmp_edge : curr_agent_plan[i].incomeEdges) {
                if (tmp_edge->valid) {
                    std::cout << " Constraint Agent " << tmp_edge->from_agent_id
                              << " at node " << tmp_edge->from_node_id << ";";
                }
            }
            break;
        }
    }
}

void ADG::printActions(
    const std::vector<std::tuple<std::string, int, double, std::string,
                                 std::pair<double, double>,
                                 std::pair<double, double>>>& actions) {
    for (const auto& action : actions) {
        std::string robot_id = std::get<0>(action);
        int time = std::get<1>(action);
        double orientation = std::get<2>(action);
        std::string type = std::get<3>(action);
        auto start = std::get<4>(action);
        auto goal = std::get<5>(action);

        std::cout << "Robot ID: " << robot_id << ", nodeID: " << time
                  << ", Orientation: " << orientation << ", Type: " << type
                  << ", Start: (" << start.first << ", " << start.second << ")"
                  << ", Goal: (" << goal.first << ", " << goal.second << ")"
                  << std::endl;
    }
}

SIM_PLAN ADG::getPlan(int agent_id) {
    SIM_PLAN sim_plan;
    std::vector<int> enque_acts;
    getAvailableNodes(agent_id, enque_acts);
    // TODO@jingtian: rethink logic here if we have data loss
    for (int enque_id : enque_acts) {
        const Action& action = graph[agent_id][enque_id].action;
        // std::pair<int, int> intStart = action.start;
        // std::pair<int, int> intEnd = action.goal;
        // std::pair<double, double> doubleStart =
        // {static_cast<double>(intStart.first),
        // static_cast<double>(intStart.second)}; std::pair<double, double>
        // doubleEnd = {static_cast<double>(intEnd.first),
        // static_cast<double>(intEnd.second)};
        int task_id;
        if (action.task_ptr == nullptr) {
            task_id = -1;
        } else {
            task_id = action.task_ptr->id;
        }
        sim_plan.emplace_back(robotIDToStartIndex[action.robot_id], enque_id,
                              action.orientation, std::string(1, action.type),
                              action.start, action.goal, task_id);
        enqueue_nodes_idx[agent_id].push_back(enque_id);
    }
    // std::cout << "For agent " << agent_id << ", enqueue size is: " <<
    // enqueue_nodes_idx[agent_id].size() << std::endl;
    return sim_plan;
}

bool ADG::updateFinishedTasks(std::shared_ptr<MobileTaskManager>& task_ptr) {
    if (curr_commit.empty() or graph.empty()) {
        return false;
    }
    // finish_tasks.clear();
    // finish_tasks.resize(num_robots);
    for (int agent_id = 0; agent_id < num_robots; agent_id++) {
        for (int node_id = 0; node_id < graph[agent_id].size(); node_id++) {
            // // Current ADGNode has a non-zero task_id, then we finish a task.
            // int curr_task = graph[agent_id][node_id].task_id;
            // if (curr_task > 0) {
            //     this->finished_tasks_[agent_id].insert(curr_task);
            // }
            if (graph[agent_id][node_id].action.type == 'P') {
                task_ptr->setTask(agent_id,
                                  graph[agent_id][node_id].action.task_ptr,
                                  DELIVER);
            } else if (graph[agent_id][node_id].action.type == 'S') {
                task_ptr->setTask(
                    agent_id, graph[agent_id][node_id].action.task_ptr,
                    DONE);
            }
        }
    }
    return true;
}

void ADG::showGraph() {
    for (size_t i = 0; i < numRobots(); i++) {
        printf("Path of agent: %lu\n", i);
        int j = 0;
        for (auto& action_node : graph[i]) {
            auto action = action_node.action;
            std::cout << "        {" << action.robot_id << ", " << action.time
                      << ", " << std::fixed << std::setprecision(1)
                      << action.orientation << ", '" << action.type << "', {"
                      << action.start.first << ", " << action.start.second
                      << "}, {" << action.goal.first << ", "
                      << action.goal.second << "}, " << action.nodeID
                      << "}. Node idx: " << action_node.node_id
                      << ", idx in graph is:" << j << ", Out edges: ";
            for (auto& tmp_out_edge : action_node.outEdges) {
                std::cout << "agent " << tmp_out_edge->to_agent_id << ", node "
                          << tmp_out_edge->to_node_id << ", status "
                          << tmp_out_edge->valid << ";";
            }
            std::cout << "\tIn edges: ";
            for (auto& tmp_in_edge : action_node.incomeEdges) {
                std::cout << "agent " << tmp_in_edge->from_agent_id << ", node "
                          << tmp_in_edge->from_node_id << ", status "
                          << tmp_in_edge->valid << ";";
            }
            std::cout << std::endl;
            j++;
        }
        printf("\n");
    }
}

bool ADG::dfs(int agent_id, int node_id,
              std::unordered_map<int, std::unordered_set<int>>& visited,
              std::unordered_map<int, std::unordered_set<int>>& recStack,
              const std::vector<std::vector<ADGNode>>& graph,
              std::unordered_map<loopNode, loopNode, NodeHash>& parent,
              std::vector<loopNode>& cycle_path) {
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
    std::unordered_map<int, std::unordered_set<int>> visited;
    std::unordered_map<int, std::unordered_set<int>> recStack;
    std::unordered_map<loopNode, loopNode, NodeHash> parent;
    std::vector<loopNode> cycle_path;

    for (int agent_id = 0; agent_id < graph.size(); ++agent_id) {
        for (const auto& node : graph[agent_id]) {
            if (visited[agent_id].count(node.node_id) == 0) {
                if (dfs(agent_id, node.node_id, visited, recStack, graph,
                        parent, cycle_path)) {
                    // Print the cycle path
                    std::cout << "Cycle detected:\n";
                    for (const auto& [aid, nid] : cycle_path) {
                        std::cout << "(Agent " << aid << ", Node " << nid
                                  << ") -> ";
                    }
                    std::cout << "(back to start)\n";
                    return true;
                }
            }
        }
    }

    std::cout << "No cycle detected.\n";
    return false;
}

void ADG::printProgress() {
    //        exit(0);
    for (int agent_id = 0; agent_id < num_robots; agent_id++) {
        std::cout << "Agent " << agent_id
                  << ", ID: " << robotIDToStartIndex[agent_id]
                  << " with plan size " << graph[agent_id].size() << ": ";
        findConstraining(agent_id);
        for (int i = 0; i <= finished_node_idx[agent_id]; i++) {
            std::cout << "#";
        }
        for (auto elem : enqueue_nodes_idx[agent_id]) {
            std::cout << '0';
        }
        int unstart;
        if (enqueue_nodes_idx[agent_id].empty()) {
            unstart = finished_node_idx[agent_id];
        } else {
            unstart = enqueue_nodes_idx[agent_id].back();
        }
        for (int i = unstart + 1; i < graph[agent_id].size(); i++) {
            std::cout << "*";
        }
        std::cout << std::endl;
    }
    // std::cerr << "Robot ID of 13 is: " << robotIDToStartIndex[13] <<
    // std::endl;
    std::cout << std::endl;
}
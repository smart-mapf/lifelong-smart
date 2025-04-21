#pragma once

#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include <map>
#include <cassert>
#include <string>
#include <sstream>
#include <iomanip>
#include <unordered_set>

#include "mobile_task.h"
#include "parser.h"

typedef std::vector<std::tuple<std::string, int, double, std::string, std::pair<double, double>, std::pair<double, double>, int>> SIM_PLAN;

struct Edge {
    int from_agent_id;
    int to_agent_id;
    int from_node_id;
    int to_node_id;
    bool valid = true; // True if this edge is not resolved, False otherwise
    Edge(int from_agent, int to_agent, int from_node, int to_node): from_agent_id(from_agent), to_agent_id(to_agent),
                                                                    from_node_id(from_node), to_node_id(to_node) {}
};

struct ADG_STATS {
    int type1EdgeCount = 0;
    int type2EdgeCount = 0;
    int moveActionCount = 0;
    int rotateActionCount = 0;
    int consecutiveMoveSequences = 0;
    int totalNodes = 0;
    std::set<std::pair<int, int>> conflict_pairs;
};

struct ADGNode {
    Action action;
//    std::vector<int> dependencies; // Indices of dependent actions (outgoing edges)
//    std::vector<int> incomingType2Edges; // Indices of incoming edges

    int node_id;
    std::vector<std::shared_ptr<Edge>> incomeEdges;
    std::vector<std::shared_ptr<Edge>> outEdges;
    bool has_valid_in_edge = true;
};

struct robotState {
    std::pair<double, double> position;
    int orient = 0;
    robotState(double x, double y): position(x, y) {}
    robotState(std::pair<double, double> pos, int theta): position(pos), orient(theta) {}
};


class ADG {
public:
    ADG(int num_robots);
    [[nodiscard]] int numRobots() const {
        return num_robots;
    }

    [[nodiscard]] int numNodes() const {
        return total_nodes_cnt;
    }

    std::pair<std::map<int, std::string>, std::map<std::string, int>> createRobotIDToStartIndexMaps();
    bool createRobotIDToStartIndexMaps(std::string& robot_id_str);
    bool getAvailableNodes(int robot_id, std::vector<int>& available_nodes);
    bool updateFinishedNode(int robot_id, int node_id);
    // void setEnqueueNodes(int robot_id, std::vector<int>& enqueue_nodes);
    std::vector<robotState> computeCommitCut(int num_enqueue_node);
    void addMAPFPlan(const std::vector<std::vector<Action>>& plans);
    SIM_PLAN getPlan(int agent_id);
    std::pair<double, double> getRobotPosition(int agent_id) {
        return robot_states[agent_id].position;
    }
    bool isTaskNode(int robot_id, int node_id);


    std::pair<double, double> getActionGoal(int agent_id, int node_id)
    {
        return std::make_pair(graph[agent_id][node_id].action.goal.first, graph[agent_id][node_id].action.goal.second);
    }

    void printProgress()
    {
//        exit(0);
        for (int agent_id = 0; agent_id < num_robots; agent_id++) {
            std::cout << "Agent " << agent_id << ", ID: " << robotIDToStartIndex[agent_id] << " with plan size " << graph[agent_id].size() << ": ";
            findConstraining(agent_id);
            for (int i = 0; i <= finished_node_idx[agent_id]; i++) {
                std::cout << "#";
            }
            for (auto elem: enqueue_nodes_idx[agent_id]) {
                std::cout << '0';
            }
            int unstart;
            if (enqueue_nodes_idx[agent_id].empty()) {
                unstart = finished_node_idx[agent_id];
            } else {
                unstart = enqueue_nodes_idx[agent_id].back();
            }
            for (int i = unstart+1; i < graph[agent_id].size(); i++) {
                std::cout << "*";
            }
            std::cout << std::endl;
        }
        // std::cerr << "Robot ID of 13 is: " << robotIDToStartIndex[13] << std::endl;
        std::cout << std::endl;
    }

    // bool getFinishedTasks(std::vector<std::unordered_set<int>>& finish_tasks) {
    //     if (curr_commit.empty() or graph.empty()) {
    //         return false;
    //     }
    //     finish_tasks.clear();
    //     finish_tasks.resize(num_robots);
    //     for (int agent_id = 0; agent_id < num_robots; agent_id++) {
    //         for (int node_id = 0; node_id < graph[agent_id].size(); node_id++) {
    //             if (graph[agent_id][node_id].action.type == 'P' or graph[agent_id][node_id].action.type == 'S') {
    //                 finish_tasks[agent_id].insert(graph[agent_id][node_id].action.task_id);
    //             }
    //         }
    //     }
    //     return true;
    // }

    bool updateFinishedTasks(std::vector<std::unordered_set<int>>& finish_tasks, std::shared_ptr<MobileTaskManager>& task_ptr) {
        if (curr_commit.empty() or graph.empty()) {
            return false;
        }
        finish_tasks.clear();
        finish_tasks.resize(num_robots);
        for (int agent_id = 0; agent_id < num_robots; agent_id++) {
            for (int node_id = 0; node_id < graph[agent_id].size(); node_id++) {
                // if (node_id > finished_node_idx[agent_id]) {
                //     break;
                // }
                if (graph[agent_id][node_id].action.type == 'P') {
                    task_ptr->setTask(agent_id, graph[agent_id][node_id].action.task_ptr, DELIVER);
                } else if (graph[agent_id][node_id].action.type == 'S') {
                    task_ptr->setTask(agent_id, graph[agent_id][node_id].action.task_ptr, DONE);
                }
                // if (graph[agent_id][node_id].action.type == 'P' or graph[agent_id][node_id].action.type == 'S') {
                //     finish_tasks[agent_id].insert(graph[agent_id][node_id].action.task_id);
                //
                // }
            }
        }
        return true;
    }

    void showGraph() {
        for (size_t i = 0; i < numRobots(); i++) {
            printf("Path of agent: %lu\n", i);
            int j = 0;
            for (auto &action_node: graph[i]) {
                auto action = action_node.action;
                std::cout << "        {" << action.robot_id << ", " << action.time << ", "
                          << std::fixed << std::setprecision(1) << action.orientation << ", '"
                          << action.type << "', {" << action.start.first << ", " << action.start.second << "}, {"
                          << action.goal.first << ", " << action.goal.second << "}, " << action.nodeID << "}. Node idx: "
                 << action_node.node_id << ", idx in graph is:" << j << ", Out edges: ";
                for (auto &tmp_out_edge: action_node.outEdges) {
                    std::cout << "agent " << tmp_out_edge->to_agent_id << ", node " << tmp_out_edge->to_node_id << ", status " << tmp_out_edge->valid << ";";
                }
                std::cout << "\tIn edges: ";
                for (auto &tmp_in_edge: action_node.incomeEdges) {
                    std::cout << "agent " << tmp_in_edge->from_agent_id << ", node " << tmp_in_edge->from_node_id << ", status " << tmp_in_edge->valid << ";";
                }
                std::cout << std::endl;
                j++;
            }
            printf("\n");
        }
    }

private:
    void printActions(const std::vector<std::tuple<std::string, int, double, std::string, std::pair<double, double>, std::pair<double, double>>>& actions);
    void findConstraining(int robot_id);
    bool fixInconsistentIncomingEdge(std::vector<std::pair<int, int>>& commited_actions);
    // Helper DFS function
    bool dfs(int agent_id, int node_id,
             std::unordered_map<int, std::unordered_set<int>>& visited,
             std::unordered_map<int, std::unordered_set<int>>& recStack,
             const std::vector<std::vector<ADGNode>>& graph) {
        visited[agent_id].insert(node_id);
        recStack[agent_id].insert(node_id);

        const ADGNode& node = graph[agent_id][node_id];
        for (const auto& edge : node.outEdges) {
            if (!edge->valid) continue;
            int next_agent = edge->to_agent_id;
            int next_node = edge->to_node_id;

            if (visited[next_agent].count(next_node) == 0) {
                if (dfs(next_agent, next_node, visited, recStack, graph))
                    return true;
            } else if (recStack[next_agent].count(next_node)) {
                return true; // cycle detected
            }
        }

        recStack[agent_id].erase(node_id);
        return false;
    }

    bool hasCycle() {
        std::unordered_map<int, std::unordered_set<int>> visited;
        std::unordered_map<int, std::unordered_set<int>> recStack;

        for (int agent_id = 0; agent_id < graph.size(); ++agent_id) {
            for (const auto& node : graph[agent_id]) {
                if (visited[agent_id].count(node.node_id) == 0) {
                    if (dfs(agent_id, node.node_id, visited, recStack, graph))
                        return true;
                }
            }
        }

        return false;
    }

public:
    std::vector< int > finished_node_idx;
    std::vector< std::deque<int> > enqueue_nodes_idx;
    ADG_STATS adg_stats;
    bool initialized = false;
    bool get_initial_plan = false;
    std::map<int, std::string> robotIDToStartIndex;
    std::map<std::string, int> startIndexToRobotID;
    std::vector<robotState> curr_commit;

private:
    std::vector<std::vector<ADGNode>> graph;
    std::vector<std::unordered_set<int>> finished_tasks_;
    // std::vector<std::pair<double, double>> commitCut;

    int num_robots = 0;
    int total_nodes_cnt = 0;
    size_t look_ahead_dist = 0;

    std::vector<robotState> init_locs;
    std::vector<robotState> robot_states;
};
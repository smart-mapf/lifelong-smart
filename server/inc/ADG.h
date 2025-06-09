#pragma once

#include <cassert>
#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "parser.h"

typedef std::vector<
    std::tuple<std::string, int, double, std::string, std::pair<double, double>,
               std::pair<double, double>, int>>
    SIM_PLAN;

struct Edge {
    int from_agent_id;
    int to_agent_id;
    int from_node_id;
    int to_node_id;
    bool valid = true;  // True if this edge is not resolved, False otherwise
    Edge(int from_agent, int to_agent, int from_node, int to_node)
        : from_agent_id(from_agent),
          to_agent_id(to_agent),
          from_node_id(from_node),
          to_node_id(to_node) {
    }
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
    //    std::vector<int> dependencies; // Indices of dependent actions
    //    (outgoing edges) std::vector<int> incomingType2Edges; // Indices of
    //    incoming edges

    int node_id;
    std::vector<std::shared_ptr<Edge>> incomeEdges;
    std::vector<std::shared_ptr<Edge>> outEdges;
    bool has_valid_in_edge = true;
    void showNode() const {
        std::cout << "        {" << action.robot_id << ", " << action.time
                  << ", " << std::fixed << std::setprecision(1)
                  << action.orientation << ", '" << action.type << "', {"
                  << action.start.first << ", " << action.start.second << "}, {"
                  << action.goal.first << ", " << action.goal.second << "}, "
                  << action.nodeID << "}," << std::endl;
    }
};

struct robotState {
    std::pair<double, double> position;
    int orient = 0;
    robotState(double x, double y) : position(x, y) {
    }
    robotState(std::pair<double, double> pos, int theta)
        : position(pos), orient(theta) {
    }
};

class ADG {
public:
    ADG(int num_robots, int screen);
    [[nodiscard]] int numRobots() const {
        return num_robots;
    }

    [[nodiscard]] int numNodes() const {
        return total_nodes_cnt;
    }

    std::pair<std::map<int, std::string>, std::map<std::string, int>>
    createRobotIDToStartIndexMaps();
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

    std::pair<double, double> getActionGoal(int agent_id, int node_id) {
        return std::make_pair(graph[agent_id][node_id].action.goal.first,
                              graph[agent_id][node_id].action.goal.second);
    }

    bool isLastActUnfinishedTransfer(int agent_id) {
        if (graph.empty() or graph[agent_id].empty()) {
            return false;
        } else {
            if (graph[agent_id].back().action.type == 'P' and
                (graph[agent_id].size() - 1) != finished_node_idx[agent_id]) {
                return true;
            }
            return false;
        }
    }

    Location getLastLoc(int agent_id) {
        if (graph[agent_id].empty()) {
            std::cerr << "[ERROR]: Empty graph." << std::endl;
            return init_locs[agent_id].position;
        } else {
            return graph[agent_id].back().action.goal;
        }
    }

    int getNumFinishedTasks() const {
        return static_cast<int>(finished_tasks_.size());
    }

    void printProgress();

    // Return the set of finished tasks
    set<int> updateFinishedTasks();

    void showGraph();

    int getRobotCurrOrient(int agent_id) {
        if (graph[agent_id].empty()) {
            return init_locs[agent_id].orient;
        } else {
            return graph[agent_id].back().action.orientation;
        }
    }

private:
    void printActions(
        const std::vector<std::tuple<std::string, int, double, std::string,
                                     std::pair<double, double>,
                                     std::pair<double, double>>>& actions);
    void findConstraining(int robot_id);
    bool fixInconsistentIncomingEdge(
        std::vector<std::pair<int, int>>& commited_actions);
    // Helper DFS function
    using loopNode = std::pair<int, int>;
    struct NodeHash {
        std::size_t operator()(const loopNode& n) const noexcept {
            return std::hash<int>()(n.first) ^
                   (std::hash<int>()(n.second) << 1);
        }
    };
    bool dfs(int agent_id, int node_id,
             std::unordered_map<int, std::unordered_set<int>>& visited,
             std::unordered_map<int, std::unordered_set<int>>& recStack,
             const std::vector<std::vector<ADGNode>>& graph,
             std::unordered_map<loopNode, loopNode, NodeHash>& parent,
             std::vector<loopNode>& cycle_path);

    bool hasCycle();

public:
    std::vector<int> finished_node_idx;
    std::vector<std::deque<int>> enqueue_nodes_idx;
    ADG_STATS adg_stats;
    bool initialized = false;
    bool get_initial_plan = false;
    std::map<int, std::string> robotIDToStartIndex;
    std::map<std::string, int> startIndexToRobotID;
    std::vector<robotState> curr_commit;
    int screen;

private:
    std::vector<std::vector<ADGNode>> graph;
    std::unordered_set<int> finished_tasks_;
    // std::vector<std::pair<double, double>> commitCut;

    int num_robots = 0;
    int total_nodes_cnt = 0;
    size_t look_ahead_dist = 0;

    std::vector<robotState> init_locs;
    std::vector<robotState> robot_states;
};
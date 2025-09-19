#pragma once

#include "common.h"
#include "parser.h"

typedef vector<tuple<string, int, double, string, pair<double, double>,
                     pair<double, double>, int>>
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
    set<pair<int, int>> conflict_pairs;
};

struct ADGNode {
    Action action;
    //    vector<int> dependencies; // Indices of dependent actions
    //    (outgoing edges) vector<int> incomingType2Edges; // Indices of
    //    incoming edges

    // Every agent has its own node_id space
    int node_id;
    vector<shared_ptr<Edge>> incomeEdges;
    vector<shared_ptr<Edge>> outEdges;
    bool has_valid_in_edge = true;
    void showNode() const {
        cout << "        {" << action.robot_id << ", " << action.time << ", "
             << fixed << setprecision(1) << action.orientation << ", '"
             << action.type << "', {" << action.start.first << ", "
             << action.start.second << "}, {" << action.goal.first << ", "
             << action.goal.second << "}, " << action.nodeID << "}," << endl;
    }
};

struct robotState {
    pair<double, double> position;
    int orient = 0;
    robotState() : position(-1, -1), orient(-1) {
    }
    robotState(double x, double y) : position(x, y) {
    }
    robotState(pair<double, double> pos, int theta)
        : position(pos), orient(theta) {
    }
};

class ADG {
public:
    ADG(int num_robots, int screen, int look_ahead_dist);
    [[nodiscard]] int numRobots() const {
        return num_robots;
    }

    [[nodiscard]] int numNodes() const {
        return total_nodes_cnt;
    }

    bool createRobotIDToStartIndexMaps(string& robot_id_str,
                                       tuple<int, int> init_loc);
    bool getAvailableNodes(int robot_id, vector<int>& available_nodes);
    bool updateFinishedNode(int robot_id, int node_id);
    json getADGStats();
    // void setEnqueueNodes(int robot_id, vector<int>& enqueue_nodes);
    vector<robotState> computeCommitCut();
    void addMAPFPlan(const vector<vector<Action>>& plans);
    SIM_PLAN getPlan(int agent_id);
    pair<double, double> getRobotPosition(int agent_id) {
        return robot_states[agent_id].position;
    }
    bool isTaskNode(int robot_id, int node_id);

    pair<double, double> getActionGoal(int agent_id, int node_id) {
        return make_pair(graph[agent_id][node_id].action.goal.first,
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
            cerr << "[ERROR]: Empty graph." << endl;
            return init_locs[agent_id].position;
        } else {
            return graph[agent_id].back().action.goal;
        }
    }

    int getNumFinishedTasks() const {
        return n_finished_tasks;
    }

    int getNumFinishedBackupTasks() const {
        return n_finished_backup_tasks;
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

    int getNumUnfinishedActions(int agent_id);

    int getLookAheadDist() const {
        return look_ahead_dist;
    }

private:
    void printActions(
        const vector<tuple<string, int, double, string, pair<double, double>,
                           pair<double, double>>>& actions);
    void findConstraining(int robot_id);
    bool fixInconsistentIncomingEdge(vector<pair<int, int>>& commited_actions);
    void findDuplicateStarts(vector<pair<int, int>>& commited_actions) const;
    void removeUncommittedActions(
        const vector<pair<int, int>>& commited_actions);
    // Helper DFS function
    using loopNode = pair<int, int>;
    struct NodeHash {
        size_t operator()(const loopNode& n) const noexcept {
            return hash<int>()(n.first) ^ (hash<int>()(n.second) << 1);
        }
    };
    bool dfs(int agent_id, int node_id,
             unordered_map<int, unordered_set<int>>& visited,
             unordered_map<int, unordered_set<int>>& recStack,
             const vector<vector<ADGNode>>& graph,
             unordered_map<loopNode, loopNode, NodeHash>& parent,
             vector<loopNode>& cycle_path);

    bool hasCycle();

public:
    vector<int> finished_node_idx;
    vector<deque<int>> enqueue_nodes_idx;
    ADG_STATS adg_stats;
    bool initialized = false;
    bool get_initial_plan = false;
    map<int, string> robotIDToStartIndex;
    map<string, int> startIndexToRobotID;
    // Current commit cut, the robots' states of the next MAPF problem instance
    vector<robotState> curr_commit;
    int screen;
    double avg_n_rotation = 0;
    set<int> backup_tasks;

private:
    // ADG graph. graph[k][v] is the v-th action node of agent k
    vector<vector<ADGNode>> graph;
    unordered_set<int> finished_tasks_;
    int n_finished_tasks = 0;
    int n_finished_backup_tasks = 0;
    // vector<pair<double, double>> commitCut;

    int num_robots = 0;
    int total_nodes_cnt = 0;
    int look_ahead_dist = 0;
    int n_robot_init = 0;

    vector<robotState> init_locs;
    vector<robotState> robot_states;
};
#include "heuristics/LazyHeuristicTable.h"

LazyHeuristicTable::LazyHeuristicTable(const BasicGraph& G,
                                       vector<int> task_locations, int seed,
                                       string log_dir,
                                       bool save_heuristics_table)
    : HeuristicTableBase(G, task_locations, seed, log_dir,
                         save_heuristics_table) {
    std::cout << "*** Initializing lazy heuristics ***" << std::endl;
    // TODO: implement loading lazy heuristic table
    // std::ifstream myfile(this->save_path.c_str());
    // bool succ = false;
    // if (myfile.is_open()) {
    //     succ = load_heuristics_table(myfile);
    //     myfile.close();
    // }

    for (int loc : this->task_locations) {
        // Initialize an open list for each task location
        this->open_lists[loc] =
            fibonacci_heap<StateAStarNode*,
                           compare<StateAStarNode::compare_node>>();
        // Initialize an empty heuristic value for each potential pair of nodes
        this->heuristics[loc] = vector<double>(this->G.size(), WEIGHT_MAX);

        // Initialize an empty nodes table for each task location
        this->nodes[loc] =
            unordered_set<StateAStarNode*, StateAStarNode::Hasher,
                          StateAStarNode::EqNode>();
    }

    this->reset_heuristics();
}

LazyHeuristicTable::~LazyHeuristicTable() {
    heuristics.clear();

    // Empty the heaps
    for (auto& pair : open_lists) {
        pair.second.clear();
    }

    // Empty the nodes table
    for (auto& pair : nodes) {
        pair.second.clear();
    }
}

double LazyHeuristicTable::get(int goal_location, int start_location) {
    // Check if the goal locations is a task location
    if (heuristics.find(goal_location) == heuristics.end()) {
        spdlog::error("Heuristic at goal location {} does not exist.",
                      goal_location);
        exit(-1);
    }

    // Check if start location is an obstacle
    if (this->G.types[start_location] == "Obstacle") {
        spdlog::error("Heuristic at {}, which is an obstacle, does not exist.",
                      start_location);
    }

    // Check if the h val from start to goal location is computed
    if (this->heuristics.at(goal_location)[start_location] < WEIGHT_MAX) {
        return this->heuristics.at(goal_location)[start_location];
    }

    // H val is not computed yet, resume Backward Dijkstra to compute it.
    return this->lazy_dijkstra(goal_location, start_location);
}

void LazyHeuristicTable::reset_heuristics() {
    // Reset all heuristics to WEIGHT_MAX
    for (int loc : this->task_locations) {
        for (int i = 0; i < this->heuristics[loc].size(); i++) {
            this->heuristics[loc][i] = WEIGHT_MAX;
        }

        // Reset h val from task loc to itself to 0
        this->heuristics[loc][loc] = 0;
    }

    // Empty the open lists
    for (auto& pair : open_lists) {
        pair.second.clear();
    }

    // Empty the nodes table
    for (auto& pair : nodes) {
        pair.second.clear();
    }

    // Add the root node to the open list and nodes table
    for (int loc : this->task_locations) {
        State root_state(loc);
        if (this->G.consider_rotation) {
            for (auto neighbor : this->G.get_reverse_neighbors(root_state)) {
                StateAStarNode* root = new StateAStarNode(
                    State(loc, -1,
                          this->G.get_direction(neighbor.location,
                                                root_state.location)),
                    0, 0, nullptr, 0);

                // add root to heap
                root->open_handle = open_lists[loc].push(root);
                // add root to hash_table (nodes)
                nodes[loc].insert(root);
            }
        } else {
            StateAStarNode* root =
                new StateAStarNode(root_state, 0, 0, nullptr, 0);
            // add root to heap
            root->open_handle = open_lists[loc].push(root);
            // add root to hash_table (nodes)
            nodes[loc].insert(root);
        }
    }
}

double LazyHeuristicTable::lazy_dijkstra(int root_location,
                                         int start_location) {
    // Continue running dijkstra until we find the h val for start location
    while (!open_lists[root_location].empty()) {
        StateAStarNode* curr = open_lists[root_location].top();
        open_lists[root_location].pop();
        for (auto next_state : this->G.get_reverse_neighbors(curr->state)) {
            double curr_weight =
                this->G.get_weight(next_state.location, curr->state.location);
            double next_g_val;
            // Add in rotation time
            if (this->G.consider_rotation) {
                // Rotating
                if (curr->state.orientation != next_state.orientation) {
                    int degree = this->G.get_rotate_degree(
                        curr->state.orientation, next_state.orientation);
                    next_g_val = curr->g_val +
                                 degree * this->G.rotation_time * curr_weight;
                }
                // Moving
                else {
                    next_g_val = curr->g_val + curr_weight;
                }
            }
            // Not considering rotation, only moving
            else {
                next_g_val = curr->g_val + curr_weight;
            }
            StateAStarNode* next =
                new StateAStarNode(next_state, next_g_val, 0, nullptr, 0);
            auto it = nodes[root_location].find(next);
            // add the newly generated node to heap and hash table
            if (it == nodes[root_location].end()) {
                next->open_handle = open_lists[root_location].push(next);
                nodes[root_location].insert(next);
            }
            // update existing node's g_val if needed (only in the heap)
            else {
                // not needed anymore -- we already generated it before
                delete (next);
                StateAStarNode* existing_next = *it;
                if (existing_next->g_val > next_g_val) {
                    existing_next->g_val = next_g_val;
                    open_lists[root_location].increase(
                        existing_next->open_handle);
                }
            }

            // Update heuristic
            this->heuristics[root_location][next_state.location] =
                min(this->heuristics[root_location][next_state.location],
                    next_g_val);
        }

        if (curr->state.location == start_location) {
            return this->heuristics[root_location][start_location];
        }
    }

    spdlog::warn("LazyHeuristic: no path between start {} and goal {}.",
                 start_location, root_location);
    return WEIGHT_MAX;
}
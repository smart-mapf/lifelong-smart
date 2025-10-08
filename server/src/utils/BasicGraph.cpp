#include "utils/BasicGraph.h"

#include <boost/tokenizer.hpp>
#include <chrono>
#include <fstream>
#include <random>
#include <sstream>

#include "backup_planners/StateTimeAStar.h"

void BasicGraph::print_map() const {
    std::cout << "***type***" << std::endl;
    for (std::string t : types)
        std::cout << t << ",";
    std::cout << std::endl;

    std::cout << "***weights***" << std::endl;
    for (std::vector<double> n : weights) {
        for (double w : n) {
            std::cout << w << ",";
        }
        std::cout << std::endl;
    }
}

int BasicGraph::get_rotate_degree(int dir1, int dir2) const {
    if (dir1 == dir2)
        return 0;
    else if (abs(dir1 - dir2) == 1 || abs(dir1 - dir2) == 3)
        return 1;
    else
        return 2;
}

list<int> BasicGraph::get_neighbors(int v) const {
    list<int> neighbors;
    if (v < 0)
        return neighbors;

    for (int i = 0; i < 4; i++)  // move
        if (weights[v][i] < WEIGHT_MAX - 1)
            neighbors.push_back(v + move[i]);

    return neighbors;
}

list<State> BasicGraph::get_neighbors(const State& s) const {
    list<State> neighbors;
    if (s.location < 0)
        return neighbors;
    if (s.orientation >= 0) {
        neighbors.push_back(
            State(s.location, s.timestep + 1, s.orientation));  // wait
        if (weights[s.location][s.orientation] < WEIGHT_MAX - 1)
            neighbors.push_back(State(s.location + move[s.orientation],
                                      s.timestep + 1, s.orientation));  // move
        int next_orientation1 = s.orientation + 1;
        int next_orientation2 = s.orientation - 1;
        if (next_orientation2 < 0)
            next_orientation2 += 4;
        else if (next_orientation1 > 3)
            next_orientation1 -= 4;
        neighbors.push_back(
            State(s.location, s.timestep + 1, next_orientation1));  // turn left
        neighbors.push_back(State(s.location, s.timestep + 1,
                                  next_orientation2));  // turn right
    } else {
        neighbors.push_back(State(s.location, s.timestep + 1));  // wait
        for (int i = 0; i < 4; i++)                              // move
            if (weights[s.location][i] < WEIGHT_MAX - 1)
                neighbors.push_back(
                    State(s.location + move[i], s.timestep + 1));
    }
    return neighbors;
}

std::list<State> BasicGraph::get_reverse_neighbors(const State& s) const {
    std::list<State> rneighbors;
    // no wait actions
    if (s.orientation >= 0) {
        if (s.location - move[s.orientation] >= 0 &&
            s.location - move[s.orientation] < this->size() &&
            weights[s.location - move[s.orientation]][s.orientation] <
                WEIGHT_MAX - 1)
            rneighbors.push_back(State(s.location - move[s.orientation], -1,
                                       s.orientation));  // move
        int next_orientation1 = s.orientation + 1;
        int next_orientation2 = s.orientation - 1;
        if (next_orientation2 < 0)
            next_orientation2 += 4;
        else if (next_orientation1 > 3)
            next_orientation1 -= 4;
        rneighbors.push_back(
            State(s.location, -1, next_orientation1));  // turn right
        rneighbors.push_back(
            State(s.location, -1, next_orientation2));  // turn left
    } else {
        for (int i = 0; i < 4; i++)  // move
            if (s.location - move[i] >= 0 &&
                s.location - move[i] < this->size() &&
                weights[s.location - move[i]][i] < WEIGHT_MAX - 1)
                rneighbors.push_back(State(s.location - move[i]));
    }
    return rneighbors;
}

double BasicGraph::get_weight(int from, int to) const {
    if (from == to)  // wait or rotate
        return weights[from][4];
    int dir = get_direction(from, to);
    if (dir >= 0)
        return weights[from][dir];
    else
        return WEIGHT_MAX;
}

int BasicGraph::get_direction(int from, int to) const {
    for (int i = 0; i < 4; i++) {
        if (move[i] == to - from)
            return i;
    }
    if (from == to)
        return 4;
    return -1;
}

int BasicGraph::get_direction_output(int from, int to) const {
    // right, down, left, up
    int delta_move = to - from;
    // right
    if (delta_move == 1)
        return 0;
    // down
    else if (delta_move == this->cols)
        return 1;
    // left
    else if (delta_move == -1)
        return 2;
    // up
    else if (delta_move == -this->cols)
        return 3;
    else {
        std::cout << "Error in get_direction_output: " << from << " to " << to
                  << std::endl;
        return -1;
    }
}

bool BasicGraph::load_heuristics_table(std::ifstream& myfile) {
    boost::char_separator<char> sep(",");
    boost::tokenizer<boost::char_separator<char>>::iterator beg;
    std::string line;

    getline(myfile, line);  // skip "table_size"
    getline(myfile, line);
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    beg = tok.begin();
    int N = atoi((*beg).c_str());  // read number of task locations
    beg++;
    int M = atoi((*beg).c_str());  // read grid size
    if (M != this->size())
        return false;
    for (int i = 0; i < N; i++) {
        getline(myfile, line);
        int loc = atoi(line.c_str());
        getline(myfile, line);
        boost::tokenizer<boost::char_separator<char>> tok(line, sep);
        beg = tok.begin();
        std::vector<double> h_table(this->size());
        for (int j = 0; j < this->size(); j++) {
            h_table[j] = atof((*beg).c_str());
            if (h_table[j] >= DBL_MAX && types[j] != "Obstacle") {
                types[j] = "Obstacle";
                cout << "Obstacle at " << j << endl;
            }
            beg++;
        }
        heuristics[loc] = h_table;
    }
    return true;
}

void BasicGraph::save_heuristics_table(std::string fname) {
    if (this->_save_heuristics_table) {
        std::ofstream myfile;
        myfile.open(fname);
        myfile << "table_size" << std::endl
               << heuristics.size() << "," << this->size() << std::endl;

        // To preserve order, create a vector, store the heuristics there, and
        // sort the elements by key
        std::vector<std::pair<int, std::vector<double>>> heuristics_vector(
            heuristics.begin(), heuristics.end());
        std::sort(heuristics_vector.begin(), heuristics_vector.end(),
                  [](const std::pair<int, std::vector<double>>& a,
                     const std::pair<int, std::vector<double>>& b) {
                      return a.first < b.first;
                  });
        // Write the sorted heuristics to the file
        for (auto h_values : heuristics_vector) {
            myfile << h_values.first << std::endl;
            for (double h : h_values.second) {
                myfile << h << ",";
            }
            myfile << std::endl;
        }

        // for (auto h_values: heuristics)
        // {
        //     myfile << h_values.first << std::endl;
        //     cout << h_values.first << endl;
        //     for (double h : h_values.second)
        //     {
        //         myfile << h << ",";
        //         cout << h << ",";
        //     }
        //     myfile << std::endl;
        //     cout << std::endl;
        // }
        myfile.close();
    }
}

std::vector<double> BasicGraph::compute_heuristics(int root_location) {
    // std::cout << "start computing h for loc = "<< root_location <<std::endl;
    std::vector<double> res;
    if (consider_rotation) {
        // If considering rotation, we need to have 4 times the size of the grid
        res.resize(this->size() * 4, DBL_MAX);
    } else {
        // If not considering rotation, we only need the size of the grid
        res.resize(this->size(), DBL_MAX);
    }
    fibonacci_heap<StateTimeAStarNode*,
                   compare<StateTimeAStarNode::compare_node>>
        heap;
    unordered_set<StateTimeAStarNode*, StateTimeAStarNode::Hasher,
                  StateTimeAStarNode::EqNode>
        nodes;

    State root_state(root_location);
    // TODO: rotational heuristic is creating issue for some planners (lower
    // throughput compared to no rotation). Check back later.
    if (consider_rotation) {
        for (auto neighbor : get_reverse_neighbors(root_state)) {
            StateTimeAStarNode* root = new StateTimeAStarNode(
                State(root_location, -1,
                      get_direction(neighbor.location, root_state.location)),
                0, 0, nullptr, 0);
            root->open_handle = heap.push(root);  // add root to heap
            nodes.insert(root);  // add root to hash_table (nodes)
        }
    } else {
        StateTimeAStarNode* root =
            new StateTimeAStarNode(root_state, 0, 0, nullptr, 0);
        root->open_handle = heap.push(root);  // add root to heap
        nodes.insert(root);                   // add root to hash_table (nodes)
    }

    while (!heap.empty()) {
        StateTimeAStarNode* curr = heap.top();
        heap.pop();
        for (auto next_state : get_reverse_neighbors(curr->state)) {
            double curr_weight =
                get_weight(next_state.location, curr->state.location);
            double next_g_val;
            // double next_g_val = curr->g_val + curr_weight * rotation_time;
            // Add in rotation time
            if (consider_rotation) {
                // Rotating
                if (curr->state.orientation != next_state.orientation) {
                    // int degree = get_rotate_degree(next_state.orientation,
                    //                                curr->state.orientation);
                    // if (degree > 1)
                    // {
                    //     spdlog::warn("Rotating {} degrees from {} to {}",
                    //               degree * 90, curr->state.orientation,
                    //               next_state.orientation);
                    //     exit(1);
                    // }
                    // degree should always be 1
                    int degree = 1;
                    next_g_val =
                        curr->g_val + degree * rotation_time * curr_weight;
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
            StateTimeAStarNode* next =
                new StateTimeAStarNode(next_state, next_g_val, 0, nullptr, 0);
            auto it = nodes.find(next);
            if (it == nodes.end()) {  // add the newly generated node to heap
                                      // and hash table
                next->open_handle = heap.push(next);
                nodes.insert(next);
            } else {  // update existing node's g_val if needed (only in the
                      // heap)
                delete (next);  // not needed anymore -- we already generated it
                                // before
                StateTimeAStarNode* existing_next = *it;
                if (existing_next->g_val > next_g_val) {
                    existing_next->g_val = next_g_val;
                    heap.increase(existing_next->open_handle);
                }
            }
        }
    }
    // iterate over all nodes and populate the distances
    for (auto it = nodes.begin(); it != nodes.end(); it++) {
        StateTimeAStarNode* s = *it;
        if (consider_rotation) {
            int idx = s->state.location * 4 + s->state.orientation;
            res[idx] = std::min(s->g_val, res[idx]);
        } else {
            int idx = s->state.location;
            res[idx] = std::min(s->g_val, res[idx]);
        }
        delete (s);
    }
    nodes.clear();
    heap.clear();
    return res;
}

vector<double> BasicGraph::compute_pebble_motion_heuristics(int root_location) {
    // While not considering rotation, the pebble motion heuristic is the same
    // as the normal heuristic.
    if (!this->consider_rotation)
        return vector<double>(this->heuristics[root_location]);

    // std::cout << "start computing h for loc = "<< root_location <<std::endl;
    std::vector<double> res(this->size(), DBL_MAX);

    fibonacci_heap<StateTimeAStarNode*,
                   compare<StateTimeAStarNode::compare_node>>
        heap;
    unordered_set<StateTimeAStarNode*, StateTimeAStarNode::Hasher,
                  StateTimeAStarNode::EqNode>
        nodes;

    State root_state(root_location);
    StateTimeAStarNode* root =
        new StateTimeAStarNode(root_state, 0, 0, nullptr, 0);
    root->open_handle = heap.push(root);  // add root to heap
    nodes.insert(root);                   // add root to hash_table (nodes)

    while (!heap.empty()) {
        StateTimeAStarNode* curr = heap.top();
        heap.pop();
        for (auto next_state : get_reverse_neighbors(curr->state)) {
            double curr_weight =
                get_weight(next_state.location, curr->state.location);
            double next_g_val = curr->g_val + curr_weight;
            // Not considering rotation, only moving
            StateTimeAStarNode* next =
                new StateTimeAStarNode(next_state, next_g_val, 0, nullptr, 0);
            auto it = nodes.find(next);
            if (it == nodes.end()) {  // add the newly generated node to heap
                                      // and hash table
                next->open_handle = heap.push(next);
                nodes.insert(next);
            } else {  // update existing node's g_val if needed (only in the
                      // heap)
                delete (next);  // not needed anymore -- we already generated it
                                // before
                StateTimeAStarNode* existing_next = *it;
                if (existing_next->g_val > next_g_val) {
                    existing_next->g_val = next_g_val;
                    heap.increase(existing_next->open_handle);
                }
            }
        }
    }
    // iterate over all nodes and populate the distances
    for (auto it = nodes.begin(); it != nodes.end(); it++) {
        StateTimeAStarNode* s = *it;
        int idx = s->state.location;
        res[idx] = std::min(s->g_val, res[idx]);
        delete (s);
    }
    nodes.clear();
    heap.clear();
    return res;
}

double BasicGraph::get_pebble_motion_heuristic(int goal_loc,
                                               int start_loc) const {
    if (this->pebble_motion_heuristics.find(goal_loc) ==
        this->pebble_motion_heuristics.end()) {
        spdlog::error("get_pebble_motion_heuristic: error goal_loc ={}, not "
                      "find in G.pebble_motion_h",
                      goal_loc);
        exit(1);
    }
    if (this->pebble_motion_heuristics.at(goal_loc).size() <= start_loc) {
        spdlog::error(
            "get_pebble_motion_heuristic: error curr loc ={}, larger than {}",
            start_loc, this->pebble_motion_heuristics.at(goal_loc).size());
        exit(1);
    }

    return this->pebble_motion_heuristics.at(goal_loc)[start_loc];
}

int BasicGraph::get_Manhattan_distance(int loc1, int loc2) const {
    return abs(loc1 / cols - loc2 / cols) + abs(loc1 % cols - loc2 % cols);
}

void BasicGraph::copy(const BasicGraph& copy) {
    rows = copy.get_rows();
    cols = copy.get_cols();
    weights = copy.get_weights();
}

double BasicGraph::get_heuristic(int goal_loc, int start_loc,
                                 int start_ori) const {
    if (this->heuristics.find(goal_loc) == this->heuristics.end()) {
        std::cout << "error goal_loc =" << goal_loc << ", not find in G.h"
                  << std::endl;
        exit(1);
    }
    if (this->heuristics.at(goal_loc).size() <= start_loc) {
        std::cout << "error curr loc =" << start_loc << ", larger than "
                  << this->heuristics.at(goal_loc).size() << std::endl;
        exit(1);
    }
    // No start orientation is given.
    if (start_ori == -1) {
        // Considering rotation. Take the min of the four start orientations.
        if (consider_rotation) {
            double min_heuristic = DBL_MAX;
            for (int ori = 0; ori < 4; ori++) {
                double h_val =
                    this->heuristics.at(goal_loc)[start_loc * 4 + ori];
                if (h_val < min_heuristic)
                    min_heuristic = h_val;
            }
            return min_heuristic;
        }

        // Not considering rotation.
        return this->heuristics.at(goal_loc)[start_loc];
    } else {
        // Must be considering rotation
        if (!consider_rotation) {
            // std::cerr << "Error: get_heuristic called with start_ori != -1 "
            //           << "but consider_rotation is false." << std::endl;
            spdlog::warn("Error: get_heuristic called with start_ori != -1 "
                         "but consider_rotation is false.");
            exit(1);
        }
        return this->heuristics.at(goal_loc)[start_loc * 4 + start_ori];
    }
}
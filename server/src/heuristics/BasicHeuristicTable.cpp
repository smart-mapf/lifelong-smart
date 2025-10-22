#include "heuristics/BasicHeuristicTable.h"

bool HeuristicTableBase::load_heuristics_table(std::ifstream& myfile) {
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
    if (M != this->G.size())
        return false;
    for (int i = 0; i < N; i++) {
        getline(myfile, line);
        int loc = atoi(line.c_str());
        getline(myfile, line);
        boost::tokenizer<boost::char_separator<char>> tok(line, sep);
        beg = tok.begin();
        std::vector<double> h_table(this->G.size());
        for (int j = 0; j < this->G.size(); j++) {
            h_table[j] = atof((*beg).c_str());
            // if (h_table[j] >= WEIGHT_MAX && this->G.types[j] != "Obstacle") {
            //     this->G.types[j] = "Obstacle";
            //     cout << "Obstacle at " << j << endl;
            // }
            beg++;
        }
        heuristics[loc] = h_table;
    }
    return true;
}

void HeuristicTableBase::save_heuristics_table(string fname) {
    std::ofstream myfile;
    myfile.open(fname);
    myfile << "table_size" << std::endl
           << heuristics.size() << "," << this->G.size() << std::endl;

    // To preserve order, create a vector, store the heuristics there, and sort
    // the elements by key
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

    myfile.close();
}

BasicHeuristicTable::BasicHeuristicTable(const BasicGraph& G,
                                         vector<int> task_locations, int seed,
                                         string log_dir,
                                         bool save_heuristics_table)
    : HeuristicTableBase(G, task_locations, seed, log_dir,
                         save_heuristics_table) {
    spdlog::info("*** Computing heuristics ***");
    clock_t t = std::clock();

    std::ifstream myfile(this->save_path.c_str());
    bool succ = false;
    if (myfile.is_open()) {
        succ = load_heuristics_table(myfile);
        myfile.close();
    }

    if (!succ) {
        this->reset_heuristics();
    }

    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    spdlog::info("Done! ({} s)", runtime);
}

double BasicHeuristicTable::get(int goal_location, int start_location) {
    if (heuristics.find(goal_location) != heuristics.end()) {
        if (heuristics.at(goal_location).size() <= start_location) {
            spdlog::error("Heuristic table size {} at goal {} does not match "
                          "the graph size {}.",
                          heuristics.at(goal_location).size(), goal_location,
                          this->G.size());
            exit(-1);
        }
        return heuristics.at(goal_location)[start_location];
    } else {
        spdlog::error("Heuristic at goal location {} does not exist.",
                      goal_location);
        exit(-1);
    }
}

std::vector<double> BasicHeuristicTable::compute_heuristics(int root_location) {
    // std::cout << "start computing h for loc = "<< root_location <<std::endl;
    std::vector<double> res(this->G.size(), WEIGHT_MAX);
    fibonacci_heap<StateAStarNode*, compare<StateAStarNode::compare_node>> heap;
    unordered_set<StateAStarNode*, StateAStarNode::Hasher,
                  StateAStarNode::EqNode>
        nodes;

    State root_state(root_location);
    if (this->G.consider_rotation) {
        for (auto neighbor : this->G.get_reverse_neighbors(root_state)) {
            StateAStarNode* root = new StateAStarNode(
                State(root_location, -1,
                      this->G.get_direction(neighbor.location,
                                            root_state.location)),
                0, 0, nullptr, 0);
            root->open_handle = heap.push(root);  // add root to heap
            nodes.insert(root);  // add root to hash_table (nodes)
        }
    } else {
        StateAStarNode* root = new StateAStarNode(root_state, 0, 0, nullptr, 0);
        root->open_handle = heap.push(root);  // add root to heap
        nodes.insert(root);                   // add root to hash_table (nodes)
    }

    while (!heap.empty()) {
        StateAStarNode* curr = heap.top();
        heap.pop();
        for (auto next_state : this->G.get_reverse_neighbors(curr->state)) {
            double curr_weight =
                this->G.get_weight(next_state.location, curr->state.location);
            double next_g_val;
            if (curr_weight < 0) {
                spdlog::error("Edge weight from {} to {} is negative.",
                              next_state.location, curr->state.location);
                exit(-1);
            }
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
            auto it = nodes.find(next);
            if (it == nodes.end()) {  // add the newly generated node to heap
                                      // and hash table
                next->open_handle = heap.push(next);
                nodes.insert(next);
            } else {  // update existing node's g_val if needed (only in the
                      // heap)
                delete (next);  // not needed anymore -- we already generated it
                                // before
                StateAStarNode* existing_next = *it;
                if (existing_next->g_val > next_g_val) {
                    existing_next->g_val = next_g_val;
                    heap.increase(existing_next->open_handle);
                }
            }
        }
    }
    // iterate over all nodes and populate the distances
    for (auto it = nodes.begin(); it != nodes.end(); it++) {
        StateAStarNode* s = *it;
        res[s->state.location] = std::min(s->g_val, res[s->state.location]);
        delete (s);
    }
    nodes.clear();
    heap.clear();
    return res;
}

void BasicHeuristicTable::reset_heuristics() {
    heuristics.clear();
    // Compute heuristics for all task locations
    for (int loc : task_locations) {
        heuristics[loc] = compute_heuristics(loc);
    }
    // cout << this->save_path << endl;
    if (this->_save_heuristics_table)
        save_heuristics_table(this->save_path.string());
}

GuidePathHVal BasicHeuristicTable::get_guide_path_h(int goal_location,
                                                    int start_location,
                                                    const Path& g_path) {
    GuidePathHVal res;

    // For all points on the guide path, select the one that is closest to
    // `start_location`.
    State closest_pt;
    int closest_idx = -1;
    res.dp = WEIGHT_MAX;
    for (int i = 0; i < g_path.size(); i++) {
        State pt = g_path[i];
        double h_val = this->get(pt.location, start_location);
        if (h_val < res.dp) {
            res.dp = h_val;
            closest_pt = pt;
            closest_idx = i;
        }

        // Stop when reached the current goal
        if (g_path[i].location == goal_location) {
            break;
        }
    }

    // Start from closest_pt, compute the distance along the guide path to the
    // `goal_location`.
    res.dg = 0;
    for (int i = closest_idx; i < g_path.size() - 1; i++) {
        // Stop if we reach the goal location. If the goal location does not
        // appear on the guide path, we will go all the way to the end of the
        // guide path.
        if (g_path[i].location == goal_location) {
            break;
        }
        double weight =
            this->G.get_weight(g_path[i].location, g_path[i + 1].location);
        if (weight > WEIGHT_MAX) {
            spdlog::error("Edge weight in guide path from {} to {} is invalid.",
                          g_path[i].location, g_path[i + 1].location);
            exit(-1);
        }
        // Print the status
        // cout << "Guide path from " << g_path[i].location << " to "
        //      << g_path[i + 1].location << " with weight " << weight
        //      << std::endl;
        res.dg += weight;
    }

    return res;
}
#include "Graph.h"

Graph::Graph(const string& map_fname, int screen)
    : map_fname(map_fname), screen(screen) {
    if (!loadMap()) {
        cerr << "Failed to load map from " << map_fname << endl;
        exit(-1);
    }
}

bool Graph::validMove(int curr, int next) const {
    if (next < 0 || next >= map_size)
        return false;
    if (my_map[next])
        return false;
    if (this->getWeight(curr, next) >= WEIGHT_MAX)
        return false;  // no edge between curr and next
    return getManhattanDistance(curr, next) < 2;
}

bool Graph::loadMapFromBench() {
    ifstream myfile(map_fname.c_str());
    if (!myfile.is_open())
        return false;
    string line;
    tokenizer<char_separator<char>>::iterator beg;
    getline(myfile, line);
    if (line[0] == 't')  // Nathan's benchmark
    {
        char_separator<char> sep(" ");
        getline(myfile, line);
        tokenizer<char_separator<char>> tok(line, sep);
        beg = tok.begin();
        beg++;
        num_of_rows = atoi((*beg).c_str());  // read number of rows
        getline(myfile, line);
        tokenizer<char_separator<char>> tok2(line, sep);
        beg = tok2.begin();
        beg++;
        num_of_cols = atoi((*beg).c_str());  // read number of cols
        getline(myfile, line);               // skip "map"
    } else                                   // my benchmark
    {
        char_separator<char> sep(",");
        tokenizer<char_separator<char>> tok(line, sep);
        beg = tok.begin();
        num_of_rows = atoi((*beg).c_str());  // read number of rows
        beg++;
        num_of_cols = atoi((*beg).c_str());  // read number of cols
    }
    map_size = num_of_cols * num_of_rows;

    // Move offsets for 4-neighbor grid
    this->move[0] = 1;             // right
    this->move[1] = -num_of_cols;  // up
    this->move[2] = -1;            // left
    this->move[3] = num_of_cols;   // down

    // Edge weights, assumed to be all 1
    this->weights.clear();
    this->weights.resize(num_of_rows * num_of_cols);

    my_map.resize(map_size, false);
    this->types.resize(map_size);
    // read map (and start/goal locations)
    for (int i = 0; i < num_of_rows; i++) {
        getline(myfile, line);
        for (int j = 0; j < num_of_cols; j++) {
            int id = linearizeCoordinate(i, j);
            this->weights[id].clear();
            this->weights[id].resize(5, WEIGHT_MAX);
            // my_map[id] = (line[j] != '.' and line[j] != 'T');
            // if (not my_map[id]) {
            //     // if it is not an obstacle, add it to free locations
            //     free_locations.push_back(id);
            //     this->weights[id][4] = 1;  // can wait here
            // }

            // Add in workstations and endpoints for warehouse map
            if (line[j] == 'w') {
                workstations.push_back(id);
                warehouse_task_locs.push_back(id);
                this->types[id] = "Workstation";
            } else if (line[j] == 'e') {
                endpoints.push_back(id);
                warehouse_task_locs.push_back(id);
                this->types[id] = "Endpoint";
            } else if (line[j] == '.') {
                this->types[id] = "Free";
                empty_locations.push_back(id);
            } else if (line[j] == 'T' || line[j] == '@') {
                this->types[id] = "Obstacle";
                my_map[id] = true;  // mark as obstacle
            }

            if (!my_map[id])
                this->weights[id][4] = 1;  // can wait here
            free_locations.push_back(id);
        }
    }
    myfile.close();

    this->setDefaultEdgeWeights();

    // initialize moves_offset array
    /*moves_offset[Graph::valid_moves_t::WAIT_MOVE] = 0;
    moves_offset[Graph::valid_moves_t::NORTH] = -num_of_cols;
    moves_offset[Graph::valid_moves_t::EAST] = 1;
    moves_offset[Graph::valid_moves_t::SOUTH] = num_of_cols;
    moves_offset[Graph::valid_moves_t::WEST] = -1;*/
    return true;
}

bool Graph::loadMapFromJson() {
    ifstream myfile(map_fname.c_str());
    if (!myfile.is_open()) {
        cerr << "Failed to open map file: " << map_fname << endl;
        return false;
    }

    json j;
    myfile >> j;

    this->num_of_rows = j["n_row"];
    this->num_of_cols = j["n_col"];
    this->map_size = num_of_rows * num_of_cols;
    this->my_map.resize(map_size, false);
    this->free_locations.clear();
    this->move[0] = 1;             // right
    this->move[1] = -num_of_cols;  // up
    this->move[2] = -1;            // left
    this->move[3] = num_of_cols;   // down

    // Edge weights, assumed to be all 1
    this->weights.clear();
    this->weights.resize(num_of_rows * num_of_cols);

    this->types.resize(map_size);

    for (int i = 0; i < this->num_of_rows; i++) {
        string line = j["layout"][i];
        for (int j = 0; j < this->num_of_cols; j++) {
            int id = linearizeCoordinate(i, j);
            this->weights[id].clear();
            this->weights[id].resize(5, WEIGHT_MAX);
            // this->my_map[id] = (line[j] != '.' and line[j] != 'T');
            // if (not this->my_map[id]) {
            //     // if it is not an obstacle, add it to free locations
            //     free_locations.push_back(id);
            //     this->weights[id][4] = 1;  // can wait here
            // }
            // Add in workstations and endpoints for warehouse map
            if (line[j] == 'w') {
                workstations.push_back(id);
                warehouse_task_locs.push_back(id);
                this->types[id] = "Workstation";
            } else if (line[j] == 'e') {
                endpoints.push_back(id);
                warehouse_task_locs.push_back(id);
                this->types[id] = "Endpoint";
            } else if (line[j] == '.') {
                this->types[id] = "Free";
                empty_locations.push_back(id);
            } else if (line[j] == 'T' || line[j] == '@') {
                this->types[id] = "Obstacle";
                my_map[id] = true;  // mark as obstacle
            }

            if (!my_map[id])
                this->weights[id][4] = 1;  // can wait here
            free_locations.push_back(id);
        }
    }

    this->setDefaultEdgeWeights();

    // Read in edge weights, if applicable
    if (j.contains("weight") && j["weight"]) {
        // Read in weights, if any
        if (j.contains("weights_matrix")) {
            vector<double> new_weights = j["weights_matrix"];
            this->update_map_weights(new_weights);
        }

        if (this->screen > 0) {
            cout << "Loaded edge weights from JSON file." << endl;
            // Print weights of the first 10 locations
            for (int i = 0;
                 i < std::min(10, static_cast<int>(this->weights.size()));
                 i++) {
                cout << "Weights for location " << i << ": ";
                for (const auto& weight : this->weights[i]) {
                    cout << weight << " ";
                }
                cout << endl;
            }
        }
    }

    myfile.close();

    if (screen > 0) {
        spdlog::info("Loaded map from JSON file: {}", map_fname);
        spdlog::info("Number of rows: {}, Number of cols: {}", num_of_rows,
                     num_of_cols);
        spdlog::info("Map size: {}", map_size);
        spdlog::info("Number of workstations: {}", workstations.size());
        spdlog::info("Number of endpoints: {}", endpoints.size());
        spdlog::info("Number of warehouse task locations: {}",
                     warehouse_task_locs.size());
        spdlog::info("Number of empty locations: {}", empty_locations.size());
    }

    return true;
}

void Graph::setDefaultEdgeWeights() {
    // Set the default edge weights assuming the graph is bi-directed
    // Set the edge weights. Here we assume the graph is undirected, setting
    // all valid edges to have weight of 1. Later we copy over the given
    // weights in the json file.
    for (int i = 0; i < this->num_of_cols * this->num_of_rows; i++) {
        if (this->my_map[i]) {
            continue;
        }
        for (int dir = 0; dir < 4; dir++) {
            int n_i = i + this->move[dir];
            if (0 <= n_i && n_i < this->num_of_cols * this->num_of_rows &&
                getManhattanDistance(i, n_i) <= 1 && !this->my_map[n_i]) {
                this->weights[i][dir] = 1;
            } else
                this->weights[i][dir] = WEIGHT_MAX;
        }
    }
}

void Graph::update_map_weights(std::vector<double>& new_weights) {
    // Read in weights
    // G_json["weights_matrix"] of length rows * cols * 7, where each entry
    // contains the weights of `right`, `down`, `left`, `up`, `wait`, `CR`, and
    // `CCR` in order.
    // Note that this order is different from the order in this->move, which is
    // `right`, `up`, `left`, `down`.
    if (this->weights.size() != this->num_of_rows * this->num_of_cols) {
        std::cout << "error weights size! weights size should be "
                  << this->num_of_rows * this->num_of_cols
                  << ", but actual =" << this->weights.size() << std::endl;
        exit(-1);
    }

    for (int i = 0; i < this->num_of_rows * this->num_of_cols; i++) {
        // Check size of weights
        if (this->weights[i].size() != 5) {
            std::cout << "error weights size at id [" << i
                      << "]! weights size should be 5"
                      << ", but actual =" << this->weights[i].size()
                      << std::endl;
            exit(-1);
        }

        // Skip if current vertex is an obstacle
        if (this->my_map[i]) {
            continue;
        }

        // Idxs
        int idx = 7 * i;

        // Wait cost
        this->weights[i][4] = new_weights[idx + 4];

        double curr_weight;  // Current weight

        // Right edge
        curr_weight = new_weights[idx];
        if (curr_weight == -1) {
            this->weights[i][0] = WEIGHT_MAX;
        } else {
            this->weights[i][0] = curr_weight;
        }

        // Down edge
        curr_weight = new_weights[idx + 1];
        if (curr_weight == -1) {
            this->weights[i][3] = WEIGHT_MAX;
        } else {
            this->weights[i][3] = curr_weight;
        }

        // Left edge
        curr_weight = new_weights[idx + 2];
        if (curr_weight == -1) {
            this->weights[i][2] = WEIGHT_MAX;
        } else {
            this->weights[i][2] = curr_weight;
        }

        // Up edge
        curr_weight = new_weights[idx + 3];
        if (curr_weight == -1) {
            this->weights[i][1] = WEIGHT_MAX;
        } else {
            this->weights[i][1] = curr_weight;
        }
    }
}

bool Graph::loadMap() {
    boost::filesystem::path map_path(map_fname);
    bool status = false;
    if (map_path.extension().string() == ".map") {
        status = loadMapFromBench();
    } else if (map_path.extension().string() == ".json") {
        status = loadMapFromJson();
    } else {
        cerr << "Unknown map file format: " << map_fname << endl;
        exit(-1);
    }

    if (!status) {
        cerr << "Failed to load map from " << map_fname << endl;
        return false;
    }

    this->computeHeuristics();
    return true;
}

// void Graph::printMap() const {
//     // std::cout << "num of rows: " << num_of_rows << " num of cols: " <<
//     // num_of_cols << std::endl;
//     for (int i = 0; i < num_of_rows; i++) {
//         for (int j = 0; j < num_of_cols; j++) {
//             if (not start_locations.empty()) {
//                 if (start_locations.front() == linearizeCoordinate(i, j)) {
//                     std::cout << "*";
//                     continue;
//                 }
//             }
//             if (this->my_map[linearizeCoordinate(i, j)])
//                 cout << '@';
//             else
//                 cout << '.';
//         }
//         cout << endl;
//     }
// }

void Graph::saveMap() const {
    ofstream myfile;
    myfile.open(map_fname);
    if (!myfile.is_open()) {
        cout << "Fail to save the map to " << map_fname << endl;
        return;
    }
    myfile << num_of_rows << "," << num_of_cols << endl;
    for (int i = 0; i < num_of_rows; i++) {
        for (int j = 0; j < num_of_cols; j++) {
            if (my_map[linearizeCoordinate(i, j)])
                myfile << "@";
            else
                myfile << ".";
        }
        myfile << endl;
    }
    myfile.close();
}

list<int> Graph::getNeighbors(int curr) const {
    list<int> neighbors;
    if (curr < 0)
        return neighbors;  // invalid location

    int candidates[4] = {curr + 1, curr - 1, curr + num_of_cols,
                         curr - num_of_cols};
    for (int next : candidates) {
        if (validMove(curr, next))
            neighbors.emplace_back(next);
    }
    return neighbors;
}

double Graph::getWeight(int from, int to) const {
    if (from == to)  // wait or rotate
        return weights[from][4];
    int dir = getDirection(from, to);
    if (dir >= 0)
        return weights[from][dir];
    else
        return WEIGHT_MAX;
}

int Graph::getDirection(int from, int to) const {
    for (int i = 0; i < 4; i++) {
        if (move[i] == to - from)
            return i;
    }
    if (from == to)
        return 4;
    return -1;
}

void Graph::computeHeuristics() {
    vector<int> h_locations;
    // Add free_location and task locations to h_locations
    h_locations.insert(h_locations.end(), free_locations.begin(),
                       free_locations.end());
    h_locations.insert(h_locations.end(), warehouse_task_locs.begin(),
                       warehouse_task_locs.end());
    if (this->screen > 0)
        cout << "Computing heuristics for " << h_locations.size()
             << " free locations." << endl;
    for (int loc : h_locations) {
        // Compute heuristics for each free location
        vector<double> heuristics_for_loc;
        vector<int> d_heuristics_for_loc;
        std::tie(heuristics_for_loc, d_heuristics_for_loc) =
            computeHeuristicsOneLoc(loc);
        this->heuristics[loc] = heuristics_for_loc;
        this->d_heuristics[loc] = d_heuristics_for_loc;
    }
    if (this->screen > 0)
        cout << "Heuristics computed." << endl;
}

// Compute backward dijkstra heuristics from a single location
tuple<vector<double>, vector<int>> Graph::computeHeuristicsOneLoc(
    int root_location) {
    struct Node {
        int location;
        double value;  // g-value (cost from root to this node)
        int duration;  // path duration.

        Node() = default;
        Node(int location, double value, int duration)
            : location(location), value(value), duration(duration) {
        }
        // the following is used to comapre nodes in the OPEN list
        struct compare_node {
            // returns true if n1 > n2 (note -- this gives us *min*-heap).
            bool operator()(const Node* n1, const Node* n2) const {
                return n1->value >= n2->value;
            }
        };  // used by OPEN (heap) to compare nodes (top of the heap has min
            // f-val, and then highest g-val)

        struct EqNode {
            bool operator()(const Node* n1, const Node* n2) const {
                return (n1 == n2) || (n1 && n2 && n1->location == n2->location);
            }
        };

        // The following is used to generate the hash value of a node
        struct Hasher {
            std::size_t operator()(const Node* n) const {
                return std::hash<int>()(n->location);
            }
        };

        fibonacci_heap<Node*, compare<Node::compare_node>>::handle_type
            open_handle;
    };

    vector<double> curr_heuristics;
    // heuristics by only considering the duration.
    // Basically, if we follow the path given by the heuristics, how many
    // timestps does it take to reach each location.
    vector<int> curr_d_heuristics;

    curr_heuristics.resize(this->map_size, DBL_MAX);
    curr_d_heuristics.resize(this->map_size, MAX_TIMESTEP);

    // std::cout << "start computing h for loc = "<< root_location <<std::endl;
    fibonacci_heap<Node*, compare<Node::compare_node>> heap;
    unordered_set<Node*, Node::Hasher, Node::EqNode> nodes;

    // if(consider_rotation)
    // {
    //     for (auto neighbor : get_reverse_neighbors(root_state))
    //     {
    //         Node* root = new Node(State(root_location, -1,
    //                 get_direction(neighbor.location, root_state.location)),
    //                 0, 0, nullptr, 0);
    //         root->open_handle = heap.push(root);  // add root to heap
    //         nodes.insert(root);       // add root to hash_table (nodes)
    //     }
    // }
    // else
    // {
    Node* root = new Node(root_location, 0, 0);
    root->open_handle = heap.push(root);  // add root to heap
    nodes.insert(root);                   // add root to hash_table (nodes)
                                          // }

    while (!heap.empty()) {
        Node* curr = heap.top();
        heap.pop();
        for (auto next_state : this->getNeighbors(curr->location)) {
            double curr_weight = this->getWeight(next_state, curr->location);
            double next_g_val;
            int next_duration;
            // // Add in rotation time
            // if (consider_rotation)
            // {
            //     // Rotating
            //     if (curr->state.orientation != next_state.orientation)
            //     {
            //         int degree = get_rotate_degree(curr->state.orientation,
            //                                        next_state.orientation);
            //         next_g_val = curr->g_val +
            //                      degree * rotation_time * curr_weight;
            //     }
            //     // Moving
            //     else
            //     {
            //         next_g_val = curr->g_val + curr_weight;
            //     }
            // }
            // Not considering rotation, only moving
            // else
            // {
            next_g_val = curr->value + curr_weight;
            next_duration = curr->duration + 1;  // increment duration by 1
            // }
            Node* next = new Node(next_state, next_g_val, next_duration);
            auto it = nodes.find(next);
            if (it == nodes.end()) {  // add the newly generated node to heap
                                      // and hash table
                next->open_handle = heap.push(next);
                nodes.insert(next);
            } else {  // update existing node's g_val if needed (only in the
                      // heap)
                delete (next);  // not needed anymore -- we already generated it
                                // before
                Node* existing_next = *it;
                if (existing_next->value > next_g_val) {
                    existing_next->value = next_g_val;
                    heap.increase(existing_next->open_handle);
                }
            }
        }
    }
    // iterate over all nodes and populate the distances
    for (auto it = nodes.begin(); it != nodes.end(); it++) {
        Node* s = *it;
        curr_heuristics[s->location] =
            std::min(s->value, curr_heuristics[s->location]);
        curr_d_heuristics[s->location] =
            std::min(s->duration, curr_d_heuristics[s->location]);
        delete (s);
    }
    nodes.clear();
    heap.clear();

    return make_tuple(curr_heuristics, curr_d_heuristics);
}
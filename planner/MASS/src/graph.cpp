#include "graph.h"

Graph::Graph(const string& map_fname, int screen)
    : map_fname(map_fname), screen(screen) {
    if (!loadMap()) {
        cerr << "Failed to load map from " << map_fname << endl;
        exit(-1);
    }
}

double Graph::getHeuristic(vector<Task> goals, int start_loc, orient start_ori,
                           int goal_id) const {
    int goal_loc = goals[goal_id].loc;
    if (heuristics.find(goal_loc) == heuristics.end()) {
        spdlog::error("Error: goal_loc = {} not found in heuristics.",
                      goal_loc);
        exit(1);
    }
    double h = heuristics.at(goal_loc)[start_loc][start_ori];
    goal_id++;
    while (goal_id < static_cast<int>(goals.size())) {
        if (heuristics.find(goals[goal_id].loc) == heuristics.end()) {
            spdlog::error("Error: goal_loc = {} not found in heuristics.",
                          goals[goal_id].loc);
            exit(1);
        }

        // Goal orientation is given
        if (goals[goal_id - 1].ori != orient::None) {
            h += heuristics.at(goals[goal_id].loc)[goals[goal_id - 1].loc]
                                                  [goals[goal_id - 1].ori];
        }
        // Goal orientation is not given, use the minimum over four orientations
        else {
            auto curr_hs =
                heuristics.at(goals[goal_id].loc)[goals[goal_id - 1].loc];
            double next_h = *std::min_element(curr_hs.begin(), curr_hs.end());
            h += next_h;
        }

        goal_id++;
    }
    return h;
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
    this->move[orient::North] = -num_of_cols;  // north
    this->move[orient::East] = 1;              // east
    this->move[orient::South] = num_of_cols;   // south
    this->move[orient::West] = -1;             // west

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

            if (!my_map[id]) {
                this->weights[id][4] = 1;  // can wait here
                free_locations.push_back(id);
            }
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
    this->move[orient::North] = -num_of_cols;  // north
    this->move[orient::East] = 1;              // east
    this->move[orient::South] = num_of_cols;   // south
    this->move[orient::West] = -1;             // west

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

            if (!my_map[id]) {
                this->weights[id][4] = 1;  // can wait here
                free_locations.push_back(id);
            }
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
    // `north`, `east`, `south`, `west`.
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
        this->weights[i][orient::None] = new_weights[idx + 4];

        double curr_weight;  // Current weight

        // Right edge (east)
        curr_weight = new_weights[idx];
        if (curr_weight == -1) {
            this->weights[i][orient::East] = WEIGHT_MAX;
        } else {
            this->weights[i][orient::East] = curr_weight;
        }

        // Down edge (south)
        curr_weight = new_weights[idx + 1];
        if (curr_weight == -1) {
            this->weights[i][orient::South] = WEIGHT_MAX;
        } else {
            this->weights[i][orient::South] = curr_weight;
        }

        // Left edge (west)
        curr_weight = new_weights[idx + 2];
        if (curr_weight == -1) {
            this->weights[i][orient::West] = WEIGHT_MAX;
        } else {
            this->weights[i][orient::West] = curr_weight;
        }

        // Up edge (north)
        curr_weight = new_weights[idx + 3];
        if (curr_weight == -1) {
            this->weights[i][orient::North] = WEIGHT_MAX;
        } else {
            this->weights[i][orient::North] = curr_weight;
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

    // Compute heuristics
    if (screen > 0) {
        spdlog::info("Computing heuristics...");
    }
    auto start_time = Time::now();
    this->computeHeuristics();
    auto end_time = Time::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
    if (screen > 0) {
        spdlog::info("Computed heuristics in {} s", duration.count() / 1000.0);
    }
    return true;
}
list<int> Graph::getNeighbors(int curr) const {
    list<int> neighbors;
    // int candidates[4] = {curr + 1, curr - 1, curr + num_of_cols,
    //                      curr - num_of_cols};
    for (int d : this->move) {
        int next = curr + d;
        if (validMove(curr, next))
            neighbors.emplace_back(next);
    }
    return neighbors;
}

/**
 * @brief Get the neighbor for heuristic function
 *
 *
 * @param curr
 * @param curr_o
 * @param neighbors
 */
void Graph::getNeighbors(
    int curr, orient curr_o,
    std::vector<std::tuple<int, orient, double>>& neighbors) const {
    int left = curr_o - 1;
    if (left < 0) {
        left += NUM_ORIENT;
    }
    auto left_o = static_cast<orient>(left);
    neighbors.emplace_back(curr, left_o, ROTATE_COST);
    int right = curr_o + 1;
    if (right >= NUM_ORIENT) {
        right -= NUM_ORIENT;
    }
    auto right_o = static_cast<orient>(right);
    neighbors.emplace_back(curr, right_o, ROTATE_COST);
    int back = curr_o + 2;
    if (back >= NUM_ORIENT) {
        back -= NUM_ORIENT;
    }
    auto back_o = static_cast<orient>(back);
    neighbors.emplace_back(curr, back_o, TURN_BACK_COST);
    // int candidates[4] = {num_of_cols, -1, -num_of_cols, 1};
    int next = curr - this->move[curr_o];
    if (validMove(curr, next))
        neighbors.emplace_back(next, curr_o, CELL_DIS / V_MAX);
}

void Graph::getInverseNeighbors(int curr, orient direct,
                                Neighbors& neighbor) const {
    int left = direct - 1;
    if (left < 0) {
        left += NUM_ORIENT;
    }
    auto left_o = static_cast<orient>(left);
    neighbor.left = std::pair<int, orient>(curr, left_o);
    int right = direct + 1;
    if (right >= NUM_ORIENT) {
        right -= NUM_ORIENT;
    }
    auto right_o = static_cast<orient>(right);
    neighbor.right = std::pair<int, orient>(curr, right_o);
    int back = direct + 2;
    if (back >= NUM_ORIENT) {
        back -= NUM_ORIENT;
    }
    auto back_o = static_cast<orient>(back);
    neighbor.back = std::pair<int, orient>(curr, back_o);

    // int candidates[4] = {-num_of_cols, 1, num_of_cols, -1};
    // int next = curr - candidates[direct];
    int next = curr - this->move[direct];
    while (validMove(curr, next)) {
        neighbor.forward_locs.push_back(next);
        curr = next;
        next = next - this->move[direct];
    }
}

void Graph::getNeighbors(int curr, orient direct, Neighbors& neighbor) const {
    int left = direct - 1;
    if (left < 0) {
        left += NUM_ORIENT;
    }
    auto left_o = static_cast<orient>(left);
    neighbor.left = std::pair<int, orient>(curr, left_o);
    int right = direct + 1;
    if (right >= NUM_ORIENT) {
        right -= NUM_ORIENT;
    }
    auto right_o = static_cast<orient>(right);
    neighbor.right = std::pair<int, orient>(curr, right_o);
    int back = direct + 2;
    if (back >= NUM_ORIENT) {
        back -= NUM_ORIENT;
    }
    auto back_o = static_cast<orient>(back);
    neighbor.back = std::pair<int, orient>(curr, back_o);

    // int candidates[4] = {-num_of_cols, 1, num_of_cols, -1};
    // int next = curr + candidates[direct];
    int next = curr + this->move[direct];
    while (validMove(curr, next)) {
        neighbor.forward_locs.push_back(next);
        curr = next;
        // next = next + candidates[direct];
        next = next + this->move[direct];
    }
}

// bool Graph::isConnected(int start, int goal) const {
//     std::queue<int> open;
//     vector<bool> closed(map_size, false);
//     open.push(start);
//     closed[start] = true;
//     while (!open.empty()) {
//         int curr = open.front();
//         open.pop();
//         if (curr == goal)
//             return true;
//         for (int next : this->getNeighbors(curr)) {
//             if (closed[next])
//                 continue;
//             open.push(next);
//             closed[next] = true;
//         }
//     }
//     return false;
// }

int Graph::randomWalk(int curr, int steps) const {
    for (int walk = 0; walk < steps; walk++) {
        list<int> l = this->getNeighbors(curr);
        vector<int> next_locations(l.cbegin(), l.cend());
        auto rng = std::default_random_engine{};
        std::shuffle(std::begin(next_locations), std::end(next_locations), rng);
        for (int next : next_locations) {
            if (this->validMove(curr, next)) {
                curr = next;
                break;
            }
        }
    }
    return curr;
}

void Graph::printMap() const {
    for (int i = 0; i < num_of_rows; i++) {
        for (int j = 0; j < num_of_cols; j++) {
            if (this->my_map[this->linearizeCoordinate(i, j)])
                cout << '@';
            else
                cout << '.';
        }
        cout << endl;
    }
}

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
            if (my_map[this->linearizeCoordinate(i, j)])
                myfile << "@";
            else
                myfile << ".";
        }
        myfile << endl;
    }
    myfile.close();
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

// Get the direction corresponding to that of this->move
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
    if (this->warehouse_task_locs.size() == 0) {
        // If no warehouse task locations, use all free locations
        h_locations = this->free_locations;
    } else {
        // Use only warehouse task locations
        h_locations = this->warehouse_task_locs;
    }
    heuristics.clear();
    size_t num_proc = std::thread::hardware_concurrency();
    num_proc = min(2, (int)num_proc);
    for (size_t i = 0; i < h_locations.size(); i += num_proc) {
        std::vector<std::thread> threads;
        size_t end_j = min(i + num_proc, h_locations.size());
        for (size_t j = i; j < end_j; j++) {
            int loc = h_locations[j];
            threads.emplace_back(&Graph::BackDijkstra, this, loc);
        }
        // std::vector<std::vector<double>> heuristics_for_loc;
        // heuristics_for_loc = BackDijkstra(loc);
        // this->heuristics[loc] = heuristics_for_loc;
        for (auto& t : threads) {
            t.join();
        }
    }
}

/**
 * @brief Help function, get the heuristic values
 *
 * @param start_loc The start location of the agent
 * @return Bool value determine if the search success
 */
bool Graph::BackDijkstra(int root_location) {
    std::vector<std::vector<double>> curr_heuristic(
        this->map_size, std::vector<double>(NUM_ORIENT));
    std::priority_queue<std::shared_ptr<Node>,
                        std::vector<std::shared_ptr<Node>>, NodeCompare>
        dij_open;
    std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual>
        dij_close_set;

    // TODO: We do not consider orientation at the goal, so we add all four
    // orientations as root
    for (int ori = 0; ori < NUM_ORIENT; ori++) {
        std::shared_ptr<Node> root_node = std::make_shared<Node>();
        root_node->g = 0.0;
        root_node->f = 0.0;
        root_node->current_point = root_location;
        root_node->curr_o = orient(ori);
        root_node->parent = nullptr;
        dij_open.push(root_node);
    }

    // std::shared_ptr<Node> root_node = std::make_shared<Node>();
    //     root_node->g = 0.0;
    //     root_node->f = 0.0;
    //     root_node->current_point = root_location;
    //     root_node->curr_o = orient::East;
    //     root_node->parent = nullptr;
    //     dij_open.push(root_node);

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
                curr_heuristic[n->current_point][n->curr_o] = n->g * 2;
                dij_close_set.insert(n);
            }
        } else {
            assert(n->current_point < curr_heuristic.size());
            assert(n->curr_o < NUM_ORIENT);
            curr_heuristic[n->current_point][n->curr_o] = n->g * 2;
            h_count++;
            dij_close_set.insert(n);
        }
        // For all the neighbor location, all need to do this operation
        Neighbors neighbors;
        this->getInverseNeighbors(n->current_point, n->curr_o, neighbors);
        if (n->parent == nullptr or n->prev_action == Action::forward) {
            // Insert two nodes, turn left and turn right
            std::shared_ptr<Node> n_left = std::make_shared<Node>();
            n_left->is_expanded = false;
            n_left->prev_action = Action::turnLeft;
            n_left->current_point = n->current_point;
            n_left->curr_o = neighbors.left.second;
            n_left->g = n->g + ROTATE_COST;
            n_left->f = n_left->g;
            n_left->parent = n;
            dij_open.push(n_left);

            std::shared_ptr<Node> n_right = std::make_shared<Node>();
            n_right->is_expanded = false;
            n_right->prev_action = Action::turnRight;
            n_right->current_point = n->current_point;
            n_right->curr_o = neighbors.right.second;
            n_right->g = n->g + ROTATE_COST;
            n_right->f = n_right->g;
            n_right->parent = n;
            dij_open.push(n_right);

            std::shared_ptr<Node> n_back = std::make_shared<Node>();
            n_back->is_expanded = false;
            n_back->prev_action = Action::turnBack;
            n_back->current_point = n->current_point;
            n_back->curr_o = neighbors.back.second;
            n_back->g = n->g + TURN_BACK_COST;
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

    std::lock_guard<std::mutex> lock(heuristic_mutex);
    heuristics[root_location] = curr_heuristic;

    // return curr_heuristic;
    return true;
}

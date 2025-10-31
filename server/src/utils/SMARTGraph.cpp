#include "utils/SMARTGraph.h"

#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <chrono>
#include <fstream>
#include <random>
#include <sstream>

#include "backup_planners/StateTimeAStar.h"

namespace fs = boost::filesystem;

int SMARTGrid::get_n_valid_edges() const {
    return this->n_valid_edges;
}

bool SMARTGrid::load_map_from_jsonstr(string G_json_str, double left_w_weight,
                                      double right_w_weight) {
    json G_json = json::parse(G_json_str);
    bool succ;
    if (G_json["weight"])
        succ =
            load_weighted_map_from_json(G_json, left_w_weight, right_w_weight);
    else
        succ = load_unweighted_map_from_json(G_json, left_w_weight,
                                             right_w_weight);
    // Analyze the aisle information if the grid type is ONE_BOT_PER_AISLE.
    if (this->grid_type == SMARTGridType::ONE_BOT_PER_AISLE) {
        this->analyze_aisle();
    }
    return succ;
}

bool SMARTGrid::load_unweighted_map_from_json(json G_json, double left_w_weight,
                                              double right_w_weight) {
    // std::cout << "*** Loading map ***" << std::endl;
    spdlog::info("*** Loading map ***");
    clock_t t = std::clock();

    // Read in n_row, n_col, n_agent_loc, maxtime
    this->rows = G_json["n_row"];
    this->cols = G_json["n_col"];
    int num_endpoints, agent_num, maxtime;
    num_endpoints = G_json["n_endpoint"];
    agent_num = G_json["n_agent_loc"];
    maxtime = G_json["maxtime"];
    this->move[0] = 1;      // right
    this->move[1] = -cols;  // up
    this->move[2] = -1;     // left
    this->move[3] = cols;   // down
    this->map_name = G_json["name"];

    this->types.resize(this->rows * this->cols);
    this->weights.clear();
    this->weights.resize(this->rows * this->cols);
    string line;

    for (int i = 0; i < this->rows; i++) {
        // getline(myfile, line);
        line = G_json["layout"][i];
        for (int j = 0; j < this->cols; j++) {
            int id = this->cols * i + j;
            this->weights[id].clear();
            this->weights[id].resize(5, WEIGHT_MAX);

            // All non-obstacle locations are free locations
            if (line[j] != '@' && line[j] != 'T') {
                this->free_locations.push_back(id);
            }

            if (line[j] == '@' || line[j] == 'T')  // obstacle
            {
                this->types[id] = CellType::OBSTACLE;
            } else if (line[j] == 'e')  // endpoint
            {
                this->types[id] = CellType::ENDPOINT;
                this->weights[id][4] = 1;
                this->endpoints.push_back(id);
                this->task_locations.push_back(id);
                // Under w mode, endpoints are start locations
                this->agent_home_locations.push_back(id);
            }
            // Only applies to w mode
            else if (line[j] == 'w')  // workstation
            {
                this->types[id] = CellType::WORKSTATION;
                this->weights[id][4] = 1;
                this->workstations.push_back(id);
                this->task_locations.push_back(id);

                // // Add weights to workstations s.t. one side of the
                // // workstations are more "popular" than the other
                // if (j == 0) {
                //     this->workstation_weights.push_back(left_w_weight);
                // } else {
                //     this->workstation_weights.push_back(right_w_weight);
                // }

                this->workstation_weights.push_back(1.0);

                // Under w mode, and with RHCR, agents can start from
                // anywhere except for obstacles.
                if (!this->useDummyPaths && !this->hold_endpoints) {
                    this->agent_home_locations.push_back(id);
                }
            } else if (line[j] == '.') {
                this->types[id] = CellType::FREE;
                this->weights[id][4] = 1;

                // Under w mode, and with RHCR, agents can start from
                // anywhere except for obstacles.
                if (!this->useDummyPaths && !this->hold_endpoints) {
                    this->agent_home_locations.push_back(id);
                }
            } else {
                spdlog::error(
                    "SMARTGrid::load_unweighted_map_from_json: unknown cell "
                    "type '{}' at ({}, {})",
                    line[j], i, j);
                exit(1);
            }
        }
    }

    // If endpoints and workstations are not available, use free_locations as task_locations
    if (this->endpoints.empty() && this->workstations.empty()) {
        this->task_locations = this->free_locations;
    }

    shuffle(this->agent_home_locations.begin(),
            this->agent_home_locations.end(), std::default_random_engine());
    int valid_edges = 0;
    int valid_vertices = 0;
    // Set the edge weights. Here we assume the graph is undirected, setting
    // all valid edges to have weight of 1. Later we copy over the given
    // weights in the json file.
    for (int i = 0; i < this->cols * this->rows; i++) {
        if (this->types[i] == CellType::OBSTACLE) {
            continue;
        }
        valid_vertices += 1;
        for (int dir = 0; dir < 4; dir++) {
            if (0 <= i + this->move[dir] &&
                i + this->move[dir] < this->cols * this->rows &&
                get_Manhattan_distance(i, i + this->move[dir]) <= 1 &&
                this->types[i + this->move[dir]] != CellType::OBSTACLE) {
                valid_edges += 1;
                this->weights[i][dir] = 1;
            } else
                this->weights[i][dir] = WEIGHT_MAX;
        }
    }
    this->n_valid_edges = valid_edges;
    this->n_valid_vertices = valid_vertices;

    // Done with loading the map
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;

    spdlog::info("Map size: {}x{} with {} endpoints and {} workstations.",
                 this->rows, this->cols, this->endpoints.size(),
                 this->workstations.size());
    spdlog::info("Done! ({:.2f} s)", runtime);
    return true;
}

void SMARTGrid::update_map_weights(std::vector<double>& new_weights) {
    // Read in weights
    // G_json["weights_matrix"] of length rows * cols * 7, where each 5 entry
    // contains the weights of `right`, `down`, `left`, `up`, `wait`, `CR`,
    // `CCR` in order. For now, CR and CCR are not used.
    // The costs of CR and CCR while considering rotation is the same as wait.
    // Note that this order is different from the order in this->move, which is
    // `right`, `up`, `left`, `down`.
    if (this->weights.size() != this->rows * this->cols) {
        std::cout << "error weights size! weights size should be "
                  << this->rows * this->cols
                  << ", but actual =" << this->weights.size() << std::endl;
        exit(-1);
    }

    for (int i = 0; i < this->rows * this->cols; i++) {
        // Check size of weights
        if (this->weights[i].size() != 5) {
            std::cout << "error weights size at id [" << i
                      << "]! weights size should be 5"
                      << ", but actual =" << this->weights[i].size()
                      << std::endl;
            exit(-1);
        }

        // Skip if current vertex is an obstacle
        if (this->types[i] == CellType::OBSTACLE) {
            continue;
        }

        // // For one-bot-per-aisle, skip if the current vertex is in aisle but
        // // not an aisle entry
        // if (this->grid_type == SMARTGridType::ONE_BOT_PER_AISLE &&
        //     this->in_aisle(i) && !this->is_aisle_entry(i)) {
        //     continue;
        // }

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

bool SMARTGrid::load_weighted_map_from_json(json G_json, double left_w_weight,
                                            double right_w_weight) {
    // Use load_unweighted_map_from_json to read in everything except for edge
    // weights and wait costs (technically the weights will be initialized to
    // 1). Then set the edge weights those that are given in the map json file.
    load_unweighted_map_from_json(G_json, left_w_weight, right_w_weight);

    // Read in weights, if any
    if (G_json.contains("weights_matrix")) {
        vector<double> new_weights = G_json["weights_matrix"];
        this->update_map_weights(new_weights);
    }

    // if (G_json["optimize_wait"])
    //     cout << "Optimizing all wait costs and edge weights" << endl;
    // else
    //     cout << "Optimizing one wait cost and edge weights" << endl;
    // cout << "Number of weights optimized: " << new_weights.size() << endl;
    // cout << "# valid vertices + # valid edges = " << this->n_valid_vertices
    //      << " + " << this->n_valid_edges << " = "
    //      << this->n_valid_vertices + this->n_valid_edges << endl;
    spdlog::info("# valid vertices + # valid edges = {} + {} = {}",
                 this->n_valid_vertices, this->n_valid_edges,
                 this->n_valid_vertices + this->n_valid_edges);
    // assert(j == new_weights.size());
    return true;
}

double SMARTGrid::get_avg_task_len(
    unordered_map<int, vector<double>> heuristics) const {
    double total_task_len = 0.0;
    int n_tasks = 0;

    for (auto workstation : this->workstations) {
        for (auto endpoint2 : this->endpoints) {
            total_task_len += heuristics[workstation][endpoint2];
            n_tasks += 1;
        }
    }
    return total_task_len / n_tasks;
}

void SMARTGrid::analyze_aisle() {
    // This function is applicable to the SMARTGridType::ONE_BOT_PER_AISLE.
    if (this->grid_type != SMARTGridType::ONE_BOT_PER_AISLE) {
        throw std::runtime_error(
            "analyze_aisle() can only be called for ONE_BOT_PER_AISLE grid.");
    }

    // Analyze the aisles in the grid and print the information.
    // This is a placeholder for the actual implementation.
    spdlog::info("Analyzing aisles in the SMART grid...");

    // Find all the endpoint components that are connected in the aisles.
    // For each component, print the number of endpoints and the locations.
    unordered_set<int> visited;
    for (const auto& endpoint : this->endpoints) {
        if (visited.find(endpoint) != visited.end()) {
            continue;  // Already visited this endpoint.
        }

        // Start a new component.
        set<int> component;
        component.insert(endpoint);
        visited.insert(endpoint);

        // Perform BFS to find all connected endpoints in the aisle.
        queue<int> q;
        q.push(endpoint);
        while (!q.empty()) {
            int current = q.front();
            q.pop();

            for (const auto& neighbor : get_neighbors(current)) {
                if (visited.find(neighbor) == visited.end() &&
                    this->types[neighbor] == CellType::ENDPOINT) {
                    visited.insert(neighbor);
                    component.insert(neighbor);
                    q.push(neighbor);
                }
            }
        }

        // Take the node in the compoent with the largest id (which should be
        // the "south most" endpoint) as the aisle entry point.
        // We need to set all endpoints except for the aisle entry point as
        // obstacle to prevent the MAPF planner from using them.
        // Essentially, we merge each component into a single meta-node. Rest of
        // the graph does not change. MAPF algo will search on the new
        // meta-graph.
        int aisle_entry = *max_element(component.begin(), component.end());
        this->aisle_entries.insert(aisle_entry);

        // Set up map from nodes to their aisle id.
        for (const auto& node : component) {
            this->endpt_to_aisle[node] = aisle_entry;
        }

        // Print the component information.
        spdlog::info("Aisle id {} found with {} endpoints: {}", aisle_entry,
                     component.size(), fmt::join(component, ", "));

        // Change all endpoints except for the aisle entry point as obstacles.
        // Note that we do not remove them from the `endpoints` vector, because
        // we still need to sample them as tasks
        for (const auto& node : component) {
            if (node != aisle_entry) {
                // this->types[node] = CellType::OBSTACLE;

                // Change the weights to and from the aisle entry point to
                // MAX_WEIGHT
                this->weights[node][4] = WEIGHT_MAX;  // cannot wait here
                for (int dir = 0; dir < 4; dir++) {
                    // From the node to the neighbor
                    this->weights[node][dir] = WEIGHT_MAX;

                    // From the neighbors to the node
                    int neighbor = node + this->move[dir];
                    if (neighbor >= 0 && neighbor < this->size() &&
                        this->get_Manhattan_distance(node, neighbor) <= 1) {
                        int rev_dir = this->get_direction(neighbor, node);
                        this->weights[neighbor][rev_dir] = WEIGHT_MAX;
                    }
                }
            }
        }

        this->aisles[aisle_entry] = component;
    }
}
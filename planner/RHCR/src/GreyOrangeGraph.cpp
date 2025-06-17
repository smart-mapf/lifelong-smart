#include "GreyOrangeGraph.h"

#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <chrono>
#include <fstream>
#include <random>
#include <sstream>

#include "StateTimeAStar.h"

namespace fs = boost::filesystem;

int GreyOrangeGrid::get_n_valid_edges() const {
    return this->n_valid_edges;
}

bool GreyOrangeGrid::load_map_from_jsonstr(std::string json_str) {
    json G_json = json::parse(json_str);
    if (G_json["weight"])
        return load_weighted_map_from_json(G_json);
    else
        return load_unweighted_map_from_json(G_json);
}

bool GreyOrangeGrid::load_map_from_jsonstr(std::string G_json_str,
                                           double left_w_weight,
                                           double right_w_weight) {
    throw std::runtime_error(
        "GreyOrangeGrid::load_map_from_jsonstr with w weight Not implemented");
}

void GreyOrangeGrid::parseMap(std::vector<std::vector<double>>& map_e,
                              std::vector<std::vector<double>>& map_w) {
    map_e.clear();
    map_e.resize(this->rows, vector<double>(this->cols, 0));
    map_w.clear();
    map_w.resize(this->rows, vector<double>(this->cols, 0));
    for (auto e_id : this->endpoints) {
        int r = e_id / this->cols;
        int c = e_id % this->cols;
        map_e[r][c] = 1;
    }
    for (auto w_id : this->workstations) {
        int r = w_id / this->cols;
        int c = w_id % this->cols;
        map_w[r][c] = 1;
    }
}

bool GreyOrangeGrid::load_unweighted_map_from_json(json G_json) {
    std::cout << "*** Loading map ***" << std::endl;
    clock_t t = std::clock();

    // Read in n_row, n_col, n_agent_loc, maxtime
    this->rows = G_json["n_row"];
    this->cols = G_json["n_col"];
    // int num_endpoints, agent_num, maxtime;
    // num_endpoints = G_json["n_endpoint"];
    // agent_num = G_json["n_agent_loc"];
    // maxtime = G_json["maxtime"];
    this->move[0] = 1;      // right
    this->move[1] = -cols;  // up
    this->move[2] = -1;     // left
    this->move[3] = cols;   // down
    this->map_name = G_json["name"];

    this->types.resize(this->rows * this->cols);
    this->weights.clear();
    this->weights.resize(this->rows * this->cols);
    std::string line;

    for (int i = 0; i < this->rows; i++) {
        // getline(myfile, line);
        line = G_json["layout"][i];
        for (int j = 0; j < this->cols; j++) {
            int id = this->cols * i + j;
            this->weights[id].clear();
            this->weights[id].resize(5, WEIGHT_MAX);
            if (line[j] == '@')  // obstacle
            {
                this->types[id] = "Obstacle";
            } else if (line[j] == 'g')  // endpoint
            {
                this->types[id] = "Endpoint";
                this->weights[id][4] = 1;
                this->endpoints.push_back(id);
                this->agent_home_locations.push_back(id);
            }
            // Only applies to w mode
            else if (line[j] == 'w')  // workstation
            {
                this->types[id] = "Workstation";
                this->weights[id][4] = 1;
                this->workstations.push_back(id);

                // With RHCR, agents can start from anywhere except for
                // obstacles.
                if (!this->useDummyPaths && !this->hold_endpoints) {
                    this->agent_home_locations.push_back(id);
                }
            } else if (line[j] == 'r') // queue point
            {
                this->types[id] = "Queuepoint";
                this->weights[id][4] = 1;

                // With RHCR, agents can start from anywhere except for
                // obstacles.
                if (!this->useDummyPaths && !this->hold_endpoints) {
                    this->agent_home_locations.push_back(id);
                }
            }
            else {
                this->types[id] = "Travel";
                this->weights[id][4] = 1;

                // With RHCR, agents can start from anywhere except for
                // obstacles.
                if (!this->useDummyPaths && !this->hold_endpoints) {
                    this->agent_home_locations.push_back(id);
                }
            }
        }
    }

    shuffle(this->agent_home_locations.begin(),
            this->agent_home_locations.end(), std::default_random_engine());
    int valid_edges = 0;
    int valid_vertices = 0;
    // Set the edge weights. Here we assume the graph is undirected, setting
    // all valid edges to have weight of 1. Later we copy over the given
    // weights in the json file.
    for (int i = 0; i < this->cols * this->rows; i++) {
        if (this->types[i] == "Obstacle") {
            continue;
        }
        valid_vertices += 1;
        for (int dir = 0; dir < 4; dir++) {
            if (0 <= i + this->move[dir] &&
                i + this->move[dir] < this->cols * this->rows &&
                get_Manhattan_distance(i, i + this->move[dir]) <= 1 &&
                this->types[i + this->move[dir]] != "Obstacle") {
                valid_edges += 1;
                this->weights[i][dir] = 1;
            } else
                this->weights[i][dir] = WEIGHT_MAX;
        }
    }
    this->n_valid_edges = valid_edges;
    this->n_valid_vertices = valid_vertices;

    // Read in the parking locations
    if (G_json.contains("parking_locs")) {
        this->parking_locations =
            G_json["parking_locs"].get<std::vector<int>>();
        std::cout << "Parking locations: " << this->parking_locations.size()
                  << std::endl;
    }

    // Read in the queue locations
    if (G_json.contains("queue_locs")) {
        auto queue_locs = G_json["queue_locs"];
        for (auto& loc : queue_locs.items()) {
            int id = std::stoi(loc.key());
            this->queue_locations[id] = loc.value().get<std::vector<int>>();
        }
        std::cout << "Queue locations: " << this->queue_locations.size()
                  << std::endl;
    }

    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Map size: " << this->rows << "x" << this->cols << " with ";

    std::cout << this->endpoints.size() << " endpoints (home stations) and "
              << this->workstations.size() << " workstations." << endl;

    std::cout << "Done! (" << runtime << " s)" << std::endl;
    return true;
}

void GreyOrangeGrid::update_map_weights(std::vector<double>& new_weights) {
    // Read in weights
    // G_json["weights_matrix"] of length rows * cols * 5, where each 5 entry
    // contains the weights of `right`, `down`, `left`, `up`, `wait` in order.
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
        if (this->types[i] == "Obstacle") {
            continue;
        }

        // Idxs
        int idx = 5 * i;

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

bool GreyOrangeGrid::load_weighted_map_from_json(json G_json) {
    // Use load_unweighted_map_from_json to read in everything except for edge
    // weights and wait costs (technically the weights will be initialized to
    // 1). Then set the edge weights those that are given in the map json file.
    load_unweighted_map_from_json(G_json);

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
    cout << "# valid vertices + # valid edges = " << this->n_valid_vertices
         << " + " << this->n_valid_edges << " = "
         << this->n_valid_vertices + this->n_valid_edges << endl;
    // assert(j == new_weights.size());
    return true;
}

bool GreyOrangeGrid::load_map(std::string fname, double left_w_weight,
                              double right_w_weight) {
    throw std::runtime_error(
        "GreyOrangeGrid::load_map with w weight Not implemented");

    // std::size_t pos = fname.rfind('.');  // position of the file extension
    // auto ext_name =
    //     fname.substr(pos, fname.size());  // get the name without extension
    // if (ext_name == ".grid")
    //     return load_weighted_map(fname);
    // else if (ext_name == ".map")
    //     return load_unweighted_map(fname);
    // else {
    //     std::cout << "Map file name should end with either .grid or .map. "
    //               << std::endl;
    //     return false;
    // }
}

bool GreyOrangeGrid::load_weighted_map(std::string fname) {
    throw std::runtime_error(
        "GreyOrangeGrid::load_weighted_map Not implemented");
    // std::string line;
    // std::ifstream myfile((fname).c_str());
    // if (!myfile.is_open()) {
    //     std::cout << "Map file " << fname << " does not exist. " <<
    //     std::endl; return false;
    // }

    // std::cout << "*** Loading map ***" << std::endl;
    // clock_t t = std::clock();
    // std::size_t pos = fname.rfind('.');  // position of the file extension
    // map_name = fname.substr(0, pos);     // get the name without extension
    // getline(myfile, line);               // skip the words "grid size"
    // getline(myfile, line);
    // boost::char_separator<char> sep(",");
    // boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    // boost::tokenizer<boost::char_separator<char>>::iterator beg =
    // tok.begin(); this->rows = atoi((*beg).c_str());  // read number of cols
    // beg++;
    // this->cols = atoi((*beg).c_str());  // read number of rows
    // move[0] = 1;
    // move[1] = -cols;
    // move[2] = -1;
    // move[3] = cols;

    // getline(myfile, line);  // skip the headers

    // // read tyeps and edge weights
    // this->types.resize(rows * cols);
    // this->weights.resize(rows * cols);
    // int valid_edges = 0;
    // int valid_vertices = 0;
    // for (int i = 0; i < rows * cols; i++) {
    //     getline(myfile, line);
    //     boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    //     beg = tok.begin();
    //     beg++;                                       // skip id
    //     this->types[i] = std::string(beg->c_str());  // read type
    //     beg++;
    //     if (types[i] == "Home") {
    //         valid_vertices += 1;
    //         this->agent_home_locations.push_back(i);
    //     } else if (types[i] == "Endpoint") {
    //         valid_vertices += 1;
    //         this->endpoints.push_back(i);
    //     }
    //     beg++;  // skip x
    //     beg++;  // skip y
    //     weights[i].resize(5);
    //     for (int j = 0; j < 5; j++)  // read edge weights
    //     {
    //         if (std::string(beg->c_str()) == "inf")
    //             weights[i][j] = WEIGHT_MAX;
    //         else {
    //             valid_edges += 1;
    //             weights[i][j] = std::stod(beg->c_str());
    //         }
    //         beg++;
    //     }
    // }
    // this->n_valid_edges = valid_edges;
    // this->n_valid_vertices = valid_vertices;

    // myfile.close();
    // double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    // std::cout << "Map size: " << rows << "x" << cols << " with ";
    // cout << endpoints.size() << " endpoints and " <<
    // agent_home_locations.size()
    //      << " home stations." << std::endl;
    // std::cout << "Done! (" << runtime << " s)" << std::endl;
    // return true;
}

// load map
bool GreyOrangeGrid::load_unweighted_map(std::string fname) {
    throw std::runtime_error(
        "GreyOrangeGrid::load_unweighted_map Not implemented");
    // std::string line;
    // std::ifstream myfile((fname).c_str());
    // if (!myfile.is_open()) {
    //     std::cout << "Map file " << fname << " does not exist. " <<
    //     std::endl; return false;
    // }

    // std::cout << "*** Loading map ***" << std::endl;
    // clock_t t = std::clock();
    // std::size_t pos = fname.rfind('.');  // position of the file extension
    // map_name = fname.substr(0, pos);     // get the name without extension
    // getline(myfile, line);

    // boost::char_separator<char> sep(",");
    // boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    // boost::tokenizer<boost::char_separator<char>>::iterator beg =
    // tok.begin(); rows = atoi((*beg).c_str());  // read number of rows beg++;
    // cols = atoi((*beg).c_str());  // read number of cols
    // move[0] = 1;
    // move[1] = -cols;
    // move[2] = -1;
    // move[3] = cols;

    // std::stringstream ss;
    // getline(myfile, line);
    // ss << line;
    // int num_endpoints;
    // ss >> num_endpoints;

    // int agent_num;
    // ss.clear();
    // getline(myfile, line);
    // ss << line;
    // ss >> agent_num;

    // ss.clear();
    // getline(myfile, line);
    // ss << line;
    // int maxtime;
    // ss >> maxtime;

    // // this->agents.resize(agent_num);
    // // endpoints.resize(num_endpoints + agent_num);
    // types.resize(rows * cols);
    // weights.resize(rows * cols);
    // // DeliverGoal.resize(row*col, false);
    // //  read map
    // // int ep = 0, ag = 0;

    // for (int i = 0; i < rows; i++) {
    //     getline(myfile, line);
    //     for (int j = 0; j < cols; j++) {
    //         int id = cols * i + j;
    //         weights[id].resize(5, WEIGHT_MAX);
    //         if (line[j] == '@')  // obstacle
    //         {
    //             types[id] = "Obstacle";
    //         } else if (line[j] == 'g')  // endpoint
    //         {
    //             types[id] = "Endpoint";
    //             weights[id][4] = 1;
    //             endpoints.push_back(id);
    //             this->agent_home_locations.push_back(id);
    //         } else if (line[j] == 'w')  // workstation
    //         {
    //             this->types[id] = "Workstation";
    //             this->weights[id][4] = 1;
    //             this->workstations.push_back(id);

    //             // Under w mode, and with RHCR, agents can start from
    //             // anywhere except for obstacles.
    //             if (!this->useDummyPaths && !this->hold_endpoints) {
    //                 this->agent_home_locations.push_back(id);
    //             }
    //         } else {
    //             types[id] = "Travel";
    //             weights[id][4] = 1;

    //             // Under w mode, and with RHCR, agents can start from
    //             // anywhere except for obstacles.
    //             if (!this->useDummyPaths && !this->hold_endpoints) {
    //                 this->agent_home_locations.push_back(id);
    //             }
    //         }
    //     }
    // }
    // int valid_edges = 0;
    // int valid_vertices = 0;
    // shuffle(agent_home_locations.begin(), agent_home_locations.end(),
    //         std::default_random_engine());
    // for (int i = 0; i < cols * rows; i++) {
    //     if (types[i] == "Obstacle") {
    //         continue;
    //     }
    //     valid_vertices += 1;
    //     for (int dir = 0; dir < 4; dir++) {
    //         if (0 <= i + move[dir] && i + move[dir] < cols * rows &&
    //             get_Manhattan_distance(i, i + move[dir]) <= 1 &&
    //             types[i + move[dir]] != "Obstacle") {
    //             valid_edges += 1;
    //             weights[i][dir] = 1;
    //         } else
    //             weights[i][dir] = WEIGHT_MAX;
    //     }
    // }
    // this->n_valid_edges = valid_edges;
    // this->n_valid_vertices = valid_vertices;

    // myfile.close();
    // double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    // std::cout << "Map size: " << rows << "x" << cols << " with ";
    // cout << endpoints.size() << " endpoints and " <<
    // agent_home_locations.size()
    //      << " home stations." << std::endl;
    // std::cout << "Done! (" << runtime << " s)" << std::endl;
    // return true;
}

void GreyOrangeGrid::preprocessing(bool consider_rotation,
                                   std::string log_dir) {
    std::cout << "*** PreProcessing map ***" << std::endl;
    clock_t t = std::clock();
    this->consider_rotation = consider_rotation;
    fs::path table_save_path(log_dir);
    if (consider_rotation)
        table_save_path /= map_name + "_rotation_heuristics_table.txt";
    else
        table_save_path /= map_name + "_heuristics_table.txt";
    std::ifstream myfile(table_save_path.c_str());
    bool succ = false;
    if (myfile.is_open()) {
        succ = load_heuristics_table(myfile);
        myfile.close();
    }
    if (!succ) {
        // Heuristics to endpoints
        for (auto endpoint : this->endpoints) {
            this->heuristics[endpoint] = compute_heuristics(endpoint);
        }

        // Heuristics to workstations
        for (auto workstation : this->workstations) {
            this->heuristics[workstation] = compute_heuristics(workstation);
        }

        // Heuristics to parking locations
        for (auto parking_location : this->parking_locations) {
            if (this->heuristics.find(parking_location) ==
                this->heuristics.end()) {
                this->heuristics[parking_location] =
                    compute_heuristics(parking_location);
            }
        }

        // Heuristics to queue locations
        for (auto& ele : this->queue_locations) {
            for (auto& queue_location : ele.second) {
                if (this->heuristics.find(queue_location) ==
                    this->heuristics.end()) {
                    this->heuristics[queue_location] =
                        compute_heuristics(queue_location);
                }
            }
        }
        cout << table_save_path << endl;
        save_heuristics_table(table_save_path.string());
    }

    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
}

void GreyOrangeGrid::reset_weights(bool consider_rotation, std::string log_dir,
                                   bool optimize_wait,
                                   std::vector<double>& weights) {
    // update weights
    this->update_map_weights(weights);

    this->heuristics.clear();
    std::cout << "*** reset map weights***" << std::endl;
    clock_t t = std::clock();
    this->consider_rotation = consider_rotation;
    // fs::path table_save_path(log_dir);
    // if (consider_rotation)
    // 	table_save_path /= map_name + "_rotation_heuristics_table.txt";
    // else
    // 	table_save_path /= map_name + "_heuristics_table.txt";

    for (auto endpoint : this->endpoints) {
        this->heuristics[endpoint] = compute_heuristics(endpoint);
        // std::cout << "endpoint= "<<endpoint<<", h size ="<<
        // this->heuristics[endpoint].size() <<std::endl;
    }

    std::cout << "after compute h, h size =" << this->heuristics.size()
              << ", end points size =" << this->endpoints.size() << std::endl;

    // additional heuristics to workstations
    for (auto workstation : this->workstations) {
        this->heuristics[workstation] = compute_heuristics(workstation);
        // std::cout << "workstation= "<<workstation<<", h size ="<<
        // this->heuristics[workstation].size() <<std::endl;
    }
    if (this->heuristics.size() !=
        this->endpoints.size() + this->workstations.size()) {
        std::cout << "error h size!" << std::endl;
        exit(1);
    }
    // cout << table_save_path << endl;
    // save_heuristics_table(table_save_path.string());

    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
}

double GreyOrangeGrid::get_avg_task_len(
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
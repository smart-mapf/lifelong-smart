#include "SMARTGraph.h"

#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <chrono>
#include <fstream>
#include <random>
#include <sstream>

#include "StateTimeAStar.h"

namespace fs = boost::filesystem;

bool SMARTGrid::get_r_mode() const {
    return this->r_mode;
}

bool SMARTGrid::get_w_mode() const {
    return this->w_mode;
}

int SMARTGrid::get_n_valid_edges() const {
    return this->n_valid_edges;
}

bool SMARTGrid::load_map_from_jsonstr(std::string G_json_str,
                                      double left_w_weight,
                                      double right_w_weight) {
    json G_json = json::parse(G_json_str);
    if (G_json["weight"])
        return load_weighted_map_from_json(G_json, left_w_weight,
                                           right_w_weight);
    else
        return load_unweighted_map_from_json(G_json, left_w_weight,
                                             right_w_weight);
}

void SMARTGrid::check_mode() {
    // r and w cannot both be true
    if (this->r_mode && this->w_mode) {
        throw std::invalid_argument("Map layout contains both 'r' and 'w'");
    }

    // r and w cannot both be false
    if (!this->r_mode && !this->w_mode) {
        throw std::invalid_argument(
            "Map layout does not contain either 'r' or 'w'");
    }
}

/**
 * Infer simulation mode (r or w) from map layout.
 */
void SMARTGrid::infer_sim_mode_from_map(json G_json) {
    this->r_mode = false;
    this->w_mode = false;
    std::string line;
    for (int i = 0; i < this->rows; i++) {
        line = G_json["layout"][i];
        for (int j = 0; j < this->cols; j++) {
            if (line[j] == 'r')
                this->r_mode = true;
            if (line[j] == 'w')
                this->w_mode = true;
        }
    }

    check_mode();
}

void SMARTGrid::parseMap(std::vector<std::vector<double>>& map_e,
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
void SMARTGrid::update_task_dist(std::mt19937& gen,
                                 std::string task_dist_type) {
    if (task_dist_type != "Gaussian") {
        std::cout << "task dist type [" << task_dist_type << "] not support yet"
                  << std::endl;
        exit(-1);
    }

    std::vector<std::vector<double>> map_e, map_w;

    this->parseMap(map_e, map_w);

    this->workstation_weights.clear();
    this->workstation_weights.resize(this->workstations.size(), 1.0);

    int h = this->rows;
    int w = this->cols;

    std::uniform_int_distribution<> dis_h(0, h - 1);
    std::uniform_int_distribution<> dis_w(0, w - 1);

    int center_h = dis_h(gen);
    int center_w = dis_w(gen);
    std::cout << "gaussian center = " << center_h << ", " << center_w
              << std::endl;

    std::vector<std::vector<double>> dist_full =
        getGaussian(h, w, center_h, center_w);
    // std::vector<std::vector<double>> dist_e(h, std::vector<double>(w, 0));
    // double max_val = 0;

    // for (int r = 0; r < h; ++r) {
    //     for (int c = 0; c < w; ++c) {
    //         dist_e[r][c] = dist_full[r][c] * map_e[r][c];
    //         if (dist_e[r][c] > max_val) max_val = dist_e[r][c];
    //     }
    // }

    // for (int r = 0; r < h; ++r) {
    //     for (int c = 0; c < w; ++c) {
    //         dist_e[r][c] /= max_val; // normalize
    //     }
    // }

    this->end_points_weights = generateVecEDist(map_e, dist_full);
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

    infer_sim_mode_from_map(G_json);

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
            } else if (line[j] == 'e')  // endpoint
            {
                this->types[id] = "Endpoint";
                this->weights[id][4] = 1;
                this->endpoints.push_back(id);
                this->task_locations.push_back(id);
                // Under w mode, endpoints are start locations
                if (this->w_mode) {
                    this->agent_home_locations.push_back(id);
                }
            }
            // Only applies to r mode
            else if (line[j] == 'r' && this->r_mode)  // robot rest
            {
                this->types[id] = "Home";
                this->weights[id][4] = 1;
                this->agent_home_locations.push_back(id);
            }
            // Only applies to w mode
            else if (line[j] == 'w' && this->w_mode)  // workstation
            {
                this->types[id] = "Workstation";
                this->weights[id][4] = 1;
                this->workstations.push_back(id);
                this->task_locations.push_back(id);

                // Add weights to workstations s.t. one side of the
                // workstations are more "popular" than the other
                if (j == 0) {
                    this->workstation_weights.push_back(left_w_weight);
                } else {
                    this->workstation_weights.push_back(right_w_weight);
                }

                // Under w mode, and with RHCR, agents can start from
                // anywhere except for obstacles.
                if (this->w_mode && !this->useDummyPaths &&
                    !this->hold_endpoints) {
                    this->agent_home_locations.push_back(id);
                }
            } else {
                this->types[id] = "Travel";
                this->weights[id][4] = 1;

                // Under w mode, and with RHCR, agents can start from
                // anywhere except for obstacles.
                if (this->w_mode && !this->useDummyPaths &&
                    !this->hold_endpoints) {
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

    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    // std::cout << "Map size: " << this->rows << "x" << this->cols << " with ";

    string map_info = "";
    if (this->w_mode) {
        // std::cout << this->endpoints.size() << " endpoints (home stations)
        // and "
        //           << this->workstations.size() << " workstations." << endl;
        map_info = std::to_string(this->endpoints.size()) +
                   " endpoints (home stations) and " +
                   std::to_string(this->workstations.size()) + " workstations.";
    } else if (this->r_mode) {
        // std::cout << this->endpoints.size() << " endpoints and "
        //           << this->agent_home_locations.size() << " home stations."
        //           << std::endl;
        map_info = std::to_string(this->endpoints.size()) + " endpoints and " +
                   std::to_string(this->agent_home_locations.size()) +
                   " home stations.";
    }
    spdlog::info("Map size: {}x{} with {}", this->rows, this->cols, map_info);
    spdlog::info("Done! ({:.2f} s)", runtime);
    // std::cout << "Done! (" << runtime << " s)" << std::endl;
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
        if (this->types[i] == "Obstacle") {
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

bool SMARTGrid::load_map(std::string fname, double left_w_weight,
                         double right_w_weight) {
    std::size_t pos = fname.rfind('.');  // position of the file extension
    auto ext_name =
        fname.substr(pos, fname.size());  // get the name without extension
    if (ext_name == ".grid")
        return load_weighted_map(fname);
    else if (ext_name == ".map")
        return load_unweighted_map(fname, left_w_weight, right_w_weight);
    else {
        std::cout << "Map file name should end with either .grid or .map. "
                  << std::endl;
        return false;
    }
}

bool SMARTGrid::load_weighted_map(std::string fname) {
    std::string line;
    std::ifstream myfile((fname).c_str());
    if (!myfile.is_open()) {
        std::cout << "Map file " << fname << " does not exist. " << std::endl;
        return false;
    }

    // std::cout << "*** Loading map ***" << std::endl;
    spdlog::info("*** Loading map ***");
    clock_t t = std::clock();
    std::size_t pos = fname.rfind('.');  // position of the file extension
    map_name = fname.substr(0, pos);     // get the name without extension
    getline(myfile, line);               // skip the words "grid size"
    getline(myfile, line);
    boost::char_separator<char> sep(",");
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();
    this->rows = atoi((*beg).c_str());  // read number of cols
    beg++;
    this->cols = atoi((*beg).c_str());  // read number of rows
    move[0] = 1;
    move[1] = -cols;
    move[2] = -1;
    move[3] = cols;

    getline(myfile, line);  // skip the headers

    // read tyeps and edge weights
    this->types.resize(rows * cols);
    this->weights.resize(rows * cols);
    int valid_edges = 0;
    int valid_vertices = 0;
    for (int i = 0; i < rows * cols; i++) {
        getline(myfile, line);
        boost::tokenizer<boost::char_separator<char>> tok(line, sep);
        beg = tok.begin();
        beg++;                                       // skip id
        this->types[i] = std::string(beg->c_str());  // read type
        beg++;
        if (types[i] == "Home") {
            valid_vertices += 1;
            this->agent_home_locations.push_back(i);
        } else if (types[i] == "Endpoint") {
            valid_vertices += 1;
            this->endpoints.push_back(i);
            this->task_locations.push_back(i);
        }
        beg++;  // skip x
        beg++;  // skip y
        weights[i].resize(5);
        for (int j = 0; j < 5; j++)  // read edge weights
        {
            if (std::string(beg->c_str()) == "inf")
                weights[i][j] = WEIGHT_MAX;
            else {
                valid_edges += 1;
                weights[i][j] = std::stod(beg->c_str());
            }
            beg++;
        }
    }
    this->n_valid_edges = valid_edges;
    this->n_valid_vertices = valid_vertices;

    myfile.close();
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Map size: " << rows << "x" << cols << " with ";
    cout << endpoints.size() << " endpoints and " << agent_home_locations.size()
         << " home stations." << std::endl;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
    return true;
}

// load map
bool SMARTGrid::load_unweighted_map(std::string fname, double left_w_weight,
                                    double right_w_weight) {
    std::string line;
    std::ifstream myfile((fname).c_str());
    if (!myfile.is_open()) {
        std::cout << "Map file " << fname << " does not exist. " << std::endl;
        return false;
    }

    // std::cout << "*** Loading map ***" << std::endl;
    spdlog::info("*** Loading map ***");
    clock_t t = std::clock();
    std::size_t pos = fname.rfind('.');  // position of the file extension
    map_name = fname.substr(0, pos);     // get the name without extension
    getline(myfile, line);

    boost::char_separator<char> sep(",");
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();
    rows = atoi((*beg).c_str());  // read number of rows
    beg++;
    cols = atoi((*beg).c_str());  // read number of cols
    move[0] = 1;
    move[1] = -cols;
    move[2] = -1;
    move[3] = cols;

    std::stringstream ss;
    getline(myfile, line);
    ss << line;
    int num_endpoints;
    ss >> num_endpoints;

    int agent_num;
    ss.clear();
    getline(myfile, line);
    ss << line;
    ss >> agent_num;

    ss.clear();
    getline(myfile, line);
    ss << line;
    int maxtime;
    ss >> maxtime;

    // this->agents.resize(agent_num);
    // endpoints.resize(num_endpoints + agent_num);
    types.resize(rows * cols);
    weights.resize(rows * cols);
    // DeliverGoal.resize(row*col, false);
    //  read map
    // int ep = 0, ag = 0;

    this->r_mode = false;
    this->w_mode = true;
    for (int i = 0; i < rows; i++) {
        getline(myfile, line);
        for (int j = 0; j < cols; j++) {
            int id = cols * i + j;
            weights[id].resize(5, WEIGHT_MAX);
            if (line[j] == '@')  // obstacle
            {
                types[id] = "Obstacle";
            } else if (line[j] == 'e')  // endpoint
            {
                types[id] = "Endpoint";
                weights[id][4] = 1;
                endpoints.push_back(id);
                this->task_locations.push_back(id);
                // Under w mode, endpoints are start locations
                if (this->w_mode)
                    this->agent_home_locations.push_back(id);
            }
            // Only applies to r mode
            else if (line[j] == 'r' && this->r_mode)  // robot rest
            {
                this->types[id] = "Home";
                this->weights[id][4] = 1;
                this->agent_home_locations.push_back(id);
            }
            // Only applies to w mode
            else if (line[j] == 'w' && this->w_mode)  // workstation
            {
                this->types[id] = "Workstation";
                this->weights[id][4] = 1;
                this->workstations.push_back(id);
                this->task_locations.push_back(id);

                // Add weights to workstations s.t. one side of the
                // workstations are more "popular" than the other
                if (j == 0) {
                    this->workstation_weights.push_back(left_w_weight);
                } else {
                    this->workstation_weights.push_back(right_w_weight);
                }

                // Under w mode, and with RHCR, agents can start from
                // anywhere except for obstacles.
                if (this->w_mode && !this->useDummyPaths &&
                    !this->hold_endpoints) {
                    this->agent_home_locations.push_back(id);
                }
            } else {
                types[id] = "Travel";
                weights[id][4] = 1;

                // Under w mode, and with RHCR, agents can start from
                // anywhere except for obstacles.
                if (this->w_mode && !this->useDummyPaths &&
                    !this->hold_endpoints) {
                    this->agent_home_locations.push_back(id);
                }
            }
        }
    }
    int valid_edges = 0;
    int valid_vertices = 0;
    shuffle(agent_home_locations.begin(), agent_home_locations.end(),
            std::default_random_engine());
    for (int i = 0; i < cols * rows; i++) {
        if (types[i] == "Obstacle") {
            continue;
        }
        valid_vertices += 1;
        for (int dir = 0; dir < 4; dir++) {
            if (0 <= i + move[dir] && i + move[dir] < cols * rows &&
                get_Manhattan_distance(i, i + move[dir]) <= 1 &&
                types[i + move[dir]] != "Obstacle") {
                valid_edges += 1;
                weights[i][dir] = 1;
            } else
                weights[i][dir] = WEIGHT_MAX;
        }
    }
    this->n_valid_edges = valid_edges;
    this->n_valid_vertices = valid_vertices;

    myfile.close();
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Map size: " << rows << "x" << cols << " with ";
    cout << endpoints.size() << " endpoints and " << agent_home_locations.size()
         << " home stations." << std::endl;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
    return true;
}

void SMARTGrid::preprocessing(bool consider_rotation, std::string log_dir) {
    // std::cout << "*** PreProcessing map ***" << std::endl;
    spdlog::info("*** PreProcessing map ***");
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
        for (auto endpoint : this->endpoints) {
            this->heuristics[endpoint] = compute_heuristics(endpoint);
        }

        // Under r mode, agent home location is separated from endpoints
        if (this->r_mode) {
            for (auto home : this->agent_home_locations) {
                this->heuristics[home] = compute_heuristics(home);
            }
        }
        // Under w mode, home location is endpoints but need additional
        // heuristics to workstations
        else if (this->w_mode) {
            for (auto workstation : this->workstations) {
                this->heuristics[workstation] = compute_heuristics(workstation);
            }
        }
        // cout << table_save_path << endl;
        save_heuristics_table(table_save_path.string());
    }

    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    // std::cout << "Done! (" << runtime << " s)" << std::endl;
    spdlog::info("Done! ({:.2f} s)", runtime);
}

void SMARTGrid::reset_weights(bool consider_rotation, std::string log_dir,
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

    // Under r mode, agent home location is separated from endpoints
    if (this->r_mode) {
        for (auto home : this->agent_home_locations) {
            this->heuristics[home] = compute_heuristics(home);
        }
    }
    // Under w mode, home location is endpoints but need additional
    // heuristics to workstations
    else if (this->w_mode) {
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
    }
    // cout << table_save_path << endl;
    // save_heuristics_table(table_save_path.string());

    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
}

double SMARTGrid::get_avg_task_len(
    unordered_map<int, vector<double>> heuristics) const {
    double total_task_len = 0.0;
    int n_tasks = 0;
    if (this->r_mode) {
        for (auto endpoint1 : this->endpoints) {
            for (auto endpoint2 : this->endpoints) {
                if (endpoint1 != endpoint2) {
                    total_task_len += heuristics[endpoint1][endpoint2];
                    n_tasks += 1;
                }
            }
        }
    } else if (this->w_mode) {
        for (auto workstation : this->workstations) {
            for (auto endpoint2 : this->endpoints) {
                total_task_len += heuristics[workstation][endpoint2];
                n_tasks += 1;
            }
        }
    }
    return total_task_len / n_tasks;
}
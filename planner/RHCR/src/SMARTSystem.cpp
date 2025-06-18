#include "SMARTSystem.h"

#include <algorithm>

#include "ECBS.h"
#include "LRAStar.h"
#include "PBS.h"
#include "WHCAStar.h"
#include "common.h"
#include "helper.h"

SMARTSystem::SMARTSystem(SMARTGrid &G, MAPFSolver &solver)
    : BasicSystem(G, solver), G(G) {
}

SMARTSystem::~SMARTSystem() {
}

void SMARTSystem::initialize() {
    initialize_solvers();

    starts.resize(num_of_drives);
    goal_locations.resize(num_of_drives);
    paths.resize(num_of_drives);
    finished_tasks.resize(num_of_drives);
    waited_time.resize(num_of_drives, 0);
    is_tasking.resize(num_of_drives, false);
    rotate_time.resize(num_of_drives, 0);
    is_rotating.resize(num_of_drives, false);

    // std::random_device rd;
    this->gen = mt19937(this->seed);

    if (this->G.get_w_mode()) {
        // Create workstation distribution
        // Number of weight must be the same as number of workstations
        assert(this->G.workstation_weights.size() ==
               this->G.workstations.size());

        this->workstation_dist =
            discrete_distribution<int>(this->G.workstation_weights.begin(),
                                       this->G.workstation_weights.end());
        cout << "Workstation distribution: ";
        for (auto w : this->G.workstation_weights) {
            cout << w << ", ";
        }
        cout << endl;
    }
    // bool succ = load_records();  // continue simulating from the records
    // if (succ) {
    //     std::cout << "load_records = " << succ << ", timestep = " << timestep
    //               << std::endl;
    // }
    // // exit(-1);

    // if (!succ) {
    //     timestep = 0;
    //     succ = load_locations();
    //     if (!succ) {
    //         cout << "Randomly generating initial start locations" << endl;
    //         initialize_start_locations();
    //         cout << "Randomly generating initial goal locations" << endl;
    //         initialize_goal_locations();
    //     }
    // }
}

void SMARTSystem::initialize_start_locations() {
    // Choose random start locations
    // Any non-obstacle locations can be start locations
    // Start locations should be unique
    throw std::runtime_error(
        "SMARTSystem::initialize_start_locations is not implemented. "
        "Please implement this function for SMARTSystem.");
}

void SMARTSystem::update_start_locations(
    vector<pair<double, double>> &start_locs, set<int> finished_tasks_id) {
    if (start_locs.size() != this->num_of_drives) {
        cout << "SMARTSystem::update_start_locations: start_locs size does not "
                "match num_of_drives."
             << endl;
    }

    if (screen > 0) {
        cout << "New finished tasks: ";
        for (const auto &task_id : finished_tasks_id) {
            cout << task_id << " ";
        }
        cout << endl;
    }

    if (!this->goal_locations.empty()) {
        // Remove finished tasks from goal locations
        for (int i = 0; i < this->num_of_drives; i++) {
            for (int j = 0; j < (int)goal_locations[i].size(); j++) {
                if (finished_tasks_id.find(goal_locations[i][j].id) !=
                    finished_tasks_id.end()) {
                    goal_locations[i][j].id = -1;  // reset goal id
                }
            }
        }

        // Remove goals that are already finished
        for (int i = 0; i < this->num_of_drives; i++) {
            goal_locations[i].erase(
                remove_if(goal_locations[i].begin(), goal_locations[i].end(),
                          [](const Task &t) { return t.id == -1; }),
                goal_locations[i].end());
        }
    }

    // Set new starts
    cout << "SMARTSystem::update_start_locations: "
         << "updating starts based on start_locs." << endl;
    for (int i = 0; i < this->num_of_drives; i++) {
        // Obtain the starts
        int row = static_cast<int>(start_locs[i].first);
        int col = static_cast<int>(start_locs[i].second);
        // TODO: Add support for orientation
        cout << "SMARTSystem::update_start_locations: "
             << "start_locs[" << i << "] = (" << row << ", " << col << ")"
             << endl;
        this->starts[i] = State(this->G.getCellId(row, col), 0);
    }
}

void SMARTSystem::initialize_goal_locations() {
    throw std::runtime_error(
        "SMARTSystem::initialize_goal_locations is not implemented. "
        "Please implement this function for SMARTSystem.");
}

// void SMARTSystem::update_task_dist(){
// 	this->G.update_task_dist(this->gen, this->task_dist_type);
// 	if (this->G.endpoints.size() != this->G.end_points_weights.size()){
// 		std::cout << "error in end points dist size! "<<std::endl;
// 		exit(1);
// 	}
// 	if (this->G.workstations.size() != this->G.workstation_weights.size()){
// 		std::cout << "error in end points dist size! "<<std::endl;
// 		exit(1);
// 	}

// 	this->end_points_dist = discrete_distribution<int>(
// 		this->G.end_points_weights.begin(),
// 		this->G.end_points_weights.end()
// 	);
// 	this->workstation_dist = discrete_distribution<int>(
// 		this->G.workstation_weights.begin(),
// 		this->G.workstation_weights.end()
// 	);

// }

// int SMARTSystem::sample_end_points(){
// 	int idx = this->end_points_dist(this->gen);
// 	return this->G.endpoints[idx];
// }

int SMARTSystem::sample_workstation() {
    // Sample a workstation based the given weight
    int idx = this->workstation_dist(this->gen);

    return this->G.workstations[idx];
}

int SMARTSystem::gen_next_goal(int agent_id, bool repeat_last_goal) {
    int next = -1;
    if (G.get_r_mode()) {
        next = G.endpoints[rand() % (int)G.endpoints.size()];
    }
    // Under w mode, alternate goal locations between workstations and endpoints
    else if (G.get_w_mode()) {
        if (this->next_goal_type[agent_id] == "w") {
            if (repeat_last_goal) {
                next = G.endpoints[rand() % (int)G.endpoints.size()];
                // next = this->sample_end_points();
            } else {
                next = sample_workstation();
                this->next_goal_type[agent_id] = "e";
            }
        } else if (this->next_goal_type[agent_id] == "e") {
            if (repeat_last_goal) {
                next = sample_workstation();
            } else {
                // next = this->sample_end_points();
                next = G.endpoints[rand() % (int)G.endpoints.size()];
                this->next_goal_type[agent_id] = "w";
            }
        } else {
            std::cout << "error! next goal type is not w or e, but "
                      << this->next_goal_type[agent_id] << std::endl;
            exit(1);
        }
    }
    return next;
}

void SMARTSystem::update_goal_locations() {
    if (!this->LRA_called)
        new_agents.clear();

    // Initialize next_goal_type
    if (this->next_goal_type.empty() && G.get_w_mode()) {
        for (int i = 0; i < num_of_drives; i++) {
            int start_loc = this->starts[i].location;
            // If start from workstation, next goal is endpoint
            if (std::find(G.workstations.begin(), G.workstations.end(),
                          start_loc) != G.workstations.end()) {
                this->next_goal_type.push_back("e");
            }
            // Otherwise, next goal is workstation
            else {
                this->next_goal_type.push_back("w");
            }
        }

        // // initialize end_points_weights
        // this->G.initialize_end_points_weights();
        // this->end_points_dist = discrete_distribution<int>(
        // 	this->G.end_points_weights.begin(),
        // 	this->G.end_points_weights.end()
        // );
    }

    // RHCR Algorithm
    for (int k = 0; k < num_of_drives; k++) {
        int curr = starts[k].location;  // current location
        Task goal;                      // The last goal location
        if (goal_locations[k].empty()) {
            // goal = make_tuple(curr, 0, 0);
            goal = Task(curr, -1, 0, 0);
        } else {
            goal = goal_locations[k].back();
        }
        double min_timesteps = G.get_Manhattan_distance((goal.location), curr);
        while (min_timesteps <= simulation_window)
        // The agent might finish its tasks during the next planning
        // horizon
        {
            // assign a new task
            Task next;
            if (G.types[goal.location] == "Endpoint" ||
                G.types[goal.location] == "Workstation") {
                // next = make_tuple(this->gen_next_goal(k), 0, 0);
                next = Task(this->gen_next_goal(k), -1, 0, 0);
                while (next == goal) {
                    // next = make_tuple(
                    //     this->gen_next_goal(k, true), 0, 0);
                    next = Task(this->gen_next_goal(k, true), -1, 0, 0);
                }
            } else if (G.types[goal.location] == "Travel") {
                int loc =
                    this->G.task_locations[rand() %
                                           (int)this->G.task_locations.size()];
                next = Task(loc, -1, 0, 0);
            } else {
                std::cout << "ERROR in update_goal_function()" << std::endl;
                std::cout << "The fiducial type at curr=" << curr
                          << " should not be " << G.types[curr] << std::endl;
                exit(-1);
            }
            next.id = task_id;
            task_id += 1;  // Increment the global task ID
            goal_locations[k].emplace_back(next);
            min_timesteps +=
                G.get_Manhattan_distance(next.location, goal.location);
            goal = next;
        }
    }
}

tuple<vector<double>, double, double> SMARTSystem::edge_pair_usage_mean_std(
    vector<vector<double>> &edge_usage) {
    // For each pair of valid edges (i, j) and (j, i), calculate the absolute
    // of their edge usage difference, and calculate mean and std.
    vector<double> edge_pair_usage(this->G.get_n_valid_edges() / 2, 0.0);
    int valid_edge_id = 0;
    int n_vertices = this->G.rows * this->G.cols;
    for (int i = 0; i < n_vertices; i++) {
        // Start from i+1 to ignore wait actions
        for (int j = i + 1; j < n_vertices; j++) {
            if (this->G.types[i] != "Obstacle" &&
                this->G.types[j] != "Obstacle" &&
                this->G.get_Manhattan_distance(i, j) <= 1) {
                edge_pair_usage[valid_edge_id] =
                    std::abs(edge_usage[i][j] - edge_usage[j][i]);
                valid_edge_id += 1;
            }
        }
    }
    // cout << "Number of valid edge pairs: " << edge_pair_usage.size() << endl;
    // cout << "End of valid edge pair counter: " << valid_edge_id << endl;
    double mean, std;
    std::tie(mean, std) = helper::mean_std(edge_pair_usage);
    return make_tuple(edge_pair_usage, mean, std);
}

tuple<vector<vector<vector<double>>>, vector<vector<double>>>
SMARTSystem::convert_edge_usage(vector<vector<double>> &edge_usage) {
    // [h, w, 4] matrix that stores the usage of each edge in the direction of
    // right, up, left, down
    int h = this->G.rows;
    int w = this->G.cols;
    vector<vector<vector<double>>> edge_usage_matrix(
        h, vector<vector<double>>(w, vector<double>(4, 0.0)));
    // [h, w] matrix that stores the usage of wait action in each vertex
    vector<vector<double>> vertex_wait_matrix(h, vector<double>(w, 0.0));

    // Transform edge usage to matrices above
    int n_vertices = this->G.rows * this->G.cols;
    for (int i = 0; i < n_vertices; i++) {
        // Start from i+1 to ignore wait actions
        for (int j = 0; j < n_vertices; j++) {
            int row_ = this->G.getRowCoordinate(i);
            int col_ = this->G.getColCoordinate(i);

            // Increment wait action usage matrix
            if (i == j) {
                vertex_wait_matrix[row_][col_] = edge_usage[i][j];
            }
            // Otherwise, if the edge is valid, update edge usage matrix
            else if (this->G.types[i] != "Obstacle" &&
                     this->G.types[j] != "Obstacle" &&
                     this->G.get_Manhattan_distance(i, j) <= 1) {
                // Obtain direction of edge (i, j)
                int dir = this->G.get_direction(i, j);

                // Set corresponding entry of edge usage matrix
                edge_usage_matrix[row_][col_][dir] = edge_usage[i][j];
            }
        }
    }
    return make_tuple(edge_usage_matrix, vertex_wait_matrix);
}

json SMARTSystem::summarizeResult() {
    // // Compute objective
    // double throughput = (double)this->num_of_tasks / this->simulation_time;

    // // Compute measures:
    // // 1. Variance of tile usage
    // // 2. Average number of waiting agents at each vertex
    // // 3. Average distance of the finished tasks
    // // 4. Average number of rotations over the agents. If not considering
    // //    rotations, this counts the number of shadow rotations
    // // 5. Average number of "reversed" action (not going in the direction
    // //    suggested by highway) in each timestep, weighted by absolute value
    // of
    // //    the difference of edges both directions
    // // 6. Variance of edge usage weighted by absolute value of the difference
    // //    of edges both directions
    // // and more...
    // int n_vertices = this->G.rows * this->G.cols;
    // std::vector<double> tile_usage(n_vertices, 0.0);
    // // // Adj matrix: [i,j] stores edge usage of edge from node_i to node_j
    // // std::vector<vector<double>> edge_usage(
    // //     n_vertices, std::vector<double>(n_vertices, 0.0));

    // std::vector<double> cr_usage_matrix(n_vertices, 0.0);
    // std::vector<double> ccr_usage_matrix(n_vertices, 0.0);
    // std::vector<double> vertex_wait_matrix(n_vertices, 0.0);
    // // Note: the order in edge_usage_matrix is different from RHCR. Here we
    // // have right, down, left, up, while in RHCR we have right up, left, down
    // std::vector<double> edge_usage_matrix(n_vertices * 4, 0.0);

    // std::vector<double> finished_task_len;
    // // std::vector<double> num_rev_action(this->simulation_time, 0.0);
    // std::vector<Path> no_wait_paths(this->paths.size());
    // for (int k = 0; k < num_of_drives; k++) {
    //     int path_length = this->paths[k].size();
    //     // no_wait_paths[k] = Path();
    //     for (int j = 0; j < path_length; j++) {
    //         State s = this->paths[k][j];

    //         // Record the path without wait actions
    //         if (j < path_length - 1) {
    //             State n_s = this->paths[k][j + 1];
    //             if (s.location != n_s.location &&
    //                 s.orientation != n_s.orientation) {
    //                 no_wait_paths[k].emplace_back(s);
    //             }
    //         }

    //         // Count tile usage
    //         tile_usage[s.location] += 1.0;

    //         // Infer the action and record the corresponding action usage
    //         // The planned path might go beyond the simulation window.
    //         if (j < path_length - 1 && s.timestep < this->simulation_time) {
    //             State next_s = this->paths[k][j + 1];
    //             // Wait action
    //             if (s.location == next_s.location &&
    //                 s.orientation == next_s.orientation) {
    //                 vertex_wait_matrix[s.location] += 1.0;
    //             }
    //             // Rotation
    //             else if (s.location == next_s.location &&
    //                      s.orientation != next_s.orientation) {
    //                 // Infer CR or CCR rotation
    //                 if ((s.orientation + 1) % 4 == next_s.orientation) {
    //                     cr_usage_matrix[s.location] += 1.0;
    //                 } else {
    //                     ccr_usage_matrix[s.location] += 1.0;
    //                 }
    //             }
    //             // Move action
    //             else {
    //                 int dir = this->G.get_direction_output(s.location,
    //                                                        next_s.location);
    //                 edge_usage_matrix[s.location * 4 + dir] += 1.0;
    //             }

    //             // // Edge weight from s_t to s_t+1
    //             // double t_to_t_p_1 = this->G.get_weight(
    //             //     s.location, next_s.location);
    //             // // Edge weight from s_t+1 to s_t
    //             // double t_p_1_to_t = this->G.get_weight(
    //             //     next_s.location, s.location);

    //             // Increment edge usage.
    //             // This DOES include number of wait action at each vertex
    //             (the
    //             // diagnal of the matrix), but they will be ignored while
    //             // calculating the mean and std.
    //             // edge_usage[s.location][next_s.location] += 1.0;

    //             // See if the action follows the edge direction "suggested"
    //             by
    //             // the highway system.
    //             // If the edge weight that the agent is traversing through is
    //             // larger than the direction in reverse, then the agent is
    //             NOT
    //             // following the suggested direction.
    //             // if (s.timestep < this->simulation_time &&
    //             //     s.location != next_s.location && t_to_t_p_1 >
    //             t_p_1_to_t)
    //             // {
    //             //     num_rev_action[s.timestep] += std::abs(t_to_t_p_1 -
    //             //     t_p_1_to_t);
    //             // }
    //         }
    //     }

    //     int prev_t = 0;
    //     for (auto task : this->finished_tasks[k]) {
    //         if (task.finish_t != 0) {
    //             int curr_t = task.finish_t;
    //             Path p = this->paths[k];

    //             // Calculate length of the path associated with this task
    //             double task_path_len = 0.0;
    //             for (int t = prev_t; t < curr_t - 1; t++) {
    //                 if (p[t].location != p[t + 1].location) {
    //                     task_path_len += 1.0;
    //                 }
    //             }
    //             finished_task_len.push_back(task_path_len);
    //             prev_t = curr_t;
    //         }
    //     }
    // }

    // std::vector<double> num_rotations(num_of_drives, 0.0);
    // for (int k = 0; k < this->num_of_drives; k++) {
    //     int path_length = this->paths[k].size();
    //     for (int j = 0; j < path_length; j++) {
    //         // While not considering rotations, count the number of shadow
    //         // rotations
    //         if (!this->consider_rotation) {
    //             if (j > 0 && j < path_length - 1) {
    //                 State prev_state = this->paths[k][j - 1];
    //                 State next_state = this->paths[k][j + 1];
    //                 int prev_row =
    //                     this->G.getRowCoordinate(prev_state.location);
    //                 int next_row =
    //                     this->G.getRowCoordinate(next_state.location);
    //                 int prev_col =
    //                     this->G.getColCoordinate(prev_state.location);
    //                 int next_col =
    //                     this->G.getColCoordinate(next_state.location);
    //                 // Count shadow rotations
    //                 if (prev_state.location == next_state.location) {
    //                     num_rotations[k] += 2;
    //                 } else if (abs(prev_row - next_row) == 1 &&
    //                            abs(prev_col - next_col) == 1) {
    //                     num_rotations[k] += 1;
    //                 }
    //             }
    //         }
    //         // Count actual rotations
    //         else {
    //             if (j < path_length - 1) {
    //                 State s = this->paths[k][j];
    //                 State next_s = this->paths[k][j + 1];
    //                 if (s.orientation != next_s.orientation) {
    //                     num_rotations[k] += 1.0;
    //                 }
    //             }
    //         }
    //     }
    // }

    // // // Longest common sub-path
    // // vector<int> subpath = helper::longest_common_subpath(
    // //     this->paths, this->simulation_time);

    // // Post process data
    // // Normalize tile usage s.t. they sum to 1
    // double tile_usage_sum = helper::sum(tile_usage);
    // helper::divide(tile_usage, tile_usage_sum);

    // double tile_usage_mean, tile_usage_std;
    // // double edge_pair_usage_mean, edge_pair_usage_std;
    // // vector<double> edge_pair_usage;
    // double num_wait_mean, num_wait_std;
    // double finished_len_mean, finished_len_std;
    // double num_rotations_mean, num_rotations_std;
    // // double num_rev_action_mean, num_rev_action_std;
    // double avg_task_len = this->G.get_avg_task_len(this->G.heuristics);

    // std::tie(tile_usage_mean, tile_usage_std) = helper::mean_std(tile_usage);
    // // std::tie(edge_pair_usage, edge_pair_usage_mean, edge_pair_usage_std) =
    // // edge_pair_usage_mean_std(edge_usage);
    // std::tie(num_wait_mean, num_wait_std) =
    //     helper::mean_std(vertex_wait_matrix);
    // std::tie(finished_len_mean, finished_len_std) =
    //     helper::mean_std(finished_task_len);
    // std::tie(num_rotations_mean, num_rotations_std) =
    //     helper::mean_std(num_rotations);
    // // std::tie(num_rev_action_mean, num_rev_action_std) =
    // // helper::mean_std(num_rev_action); std::tie(edge_usage_matrix,
    // // vertex_wait_matrix) = convert_edge_usage(edge_usage);

    // // Log some of the results
    // std::cout << std::endl;
    // std::cout << "Throughput: " << throughput << std::endl;
    // std::cout << "Std of tile usage: " << tile_usage_std << std::endl;
    // // std::cout << "Std of edge pair usage: " << edge_pair_usage_std <<
    // // std::endl;
    // std::cout << "Average wait at each timestep: " << num_wait_mean
    //           << std::endl;
    // std::cout << "Average path length of each finished task: "
    //           << finished_len_mean << std::endl;
    // std::cout << "Average path length of each task: " << avg_task_len
    //           << std::endl;
    // std::cout << "Average number of turns: " << num_rotations_mean <<
    // std::endl;
    // // std::cout << "Average number of reversed actions in highway: " <<
    // // num_rev_action_mean << std::endl; std::cout << "Length of longest
    // common
    // // path: " << subpath.size()
    // //           << std::endl;

    // // update_start_locations(0);
    // std::cout << std::endl << "Done!" << std::endl;
    // save_results();

    // // Create the result json object
    // json result;
    // result = {{"throughput", throughput},
    //           {"tile_usage", tile_usage},
    //           // {"edge_pair_usage", edge_pair_usage},
    //           {"edge_usage_matrix", edge_usage_matrix},
    //           {"vertex_wait_matrix", vertex_wait_matrix},
    //           {"cr_usage_matrix", cr_usage_matrix},
    //           {"ccr_usage_matrix", ccr_usage_matrix},
    //           // {"num_wait", vertex_wait_matrix},
    //           {"num_rotations", num_rotations},
    //           // {"num_rev_action", num_rev_action},
    //           {"finished_task_len", finished_task_len},
    //           {"tile_usage_mean", tile_usage_mean},
    //           {"tile_usage_std", tile_usage_std},
    //           // {"edge_pair_usage_mean", edge_pair_usage_mean},
    //           // {"edge_pair_usage_std", edge_pair_usage_std},
    //           {"num_wait_mean", num_wait_mean},
    //           {"num_wait_std", num_wait_std},
    //           {"finished_len_mean", finished_len_mean},
    //           {"finished_len_std", finished_len_std},
    //           {"num_rotations_mean", num_rotations_mean},
    //           {"num_rotations_std", num_rotations_std},
    //           // {"num_rev_action_mean", num_rev_action_mean},
    //           // {"num_rev_action_std", num_rev_action_std},
    //           // {"tasks_finished_timestep", tasks_finished_timestep},
    //           {"avg_task_len", avg_task_len},
    //           // {"congested", congested_sim},
    //           // {"longest_common_path", subpath},
    //           {"stop_at_timestep", this->timestep},
    //           {"n_mapf_calls", this->n_mapf_calls},
    //           {"n_rule_based_calls", this->n_rule_based_calls}};
    // return result;

    throw std::runtime_error(
        "SMARTSystem does not support summarizeResult. Please use "
        "summarizeCurrResult instead.");
}

json SMARTSystem::summarizeCurrResult(int summarize_interval) {
    throw std::runtime_error(
        "SMARTSystem does not support summarizeCurrResult. Please use "
        "summarizeResult instead.");
}

void SMARTSystem::set_total_sim_time(int _sim_time, int warmup_time) {
    throw std::runtime_error(
        "SMARTSystem does not support set_total_sim_time. Please use "
        "simulate() instead.");
}

json SMARTSystem::warmup(int warmup_time) {
    throw std::runtime_error(
        "SMARTSystem does not support warmup. Please use simulate() instead.");
}

json SMARTSystem::update_gg_and_step(int update_gg_interval) {
    throw std::runtime_error(
        "SMARTSystem does not support update_gg_and_step. Please use "
        "simulate() instead.");
}

json SMARTSystem::simulate(int simulation_time) {
    std::cout << "*** Simulating " << seed << " ***" << std::endl;
    initialize();

    // We assume the server is already running at this point.
    rpc::client client("127.0.0.1", this->port_number);
    // client.set_timeout(5000);  // in ms

    // Wait for the server to initialize
    int trial = 0;
    while (!client.call("is_initialized").as<bool>()) {
        if (screen > 0) {
            // printf("%d Waiting for server to initialize...\n", trial);
            trial++;
        }
    }

    while (true) {
        // Get the current simulation tick
        bool invoke_planner = client.call("invoke_planner").as<bool>();

        // Skip planning until the simulation step is a multiple of the
        // simulation window
        if (!invoke_planner) {
            continue;
        }

        cout << "Invoking planner" << endl;
        string result_message = client.call("get_location").as<string>();

        cout << "Result message: " << result_message << endl;

        auto result_json = json::parse(result_message);
        if (!result_json["initialized"].get<bool>()) {
            printf("Planner not initialized! Retrying\n");
            sleep(1);
            continue;
        }
        auto commit_cut = result_json["robots_location"]
                              .get<std::vector<std::pair<double, double>>>();
        auto new_finished_tasks_id =
            result_json["new_finished_tasks"].get<std::set<int>>();

        // Update start locations
        update_start_locations(commit_cut, new_finished_tasks_id);
        update_goal_locations();
        solve();

        auto new_mapf_plan = this->convert_path_to_smart();

        client.call("add_plan", new_mapf_plan);
    }

    // Never end. This process will be terminated by the python script.
}

// Convert the solution of RHCR to SMART format
// The format is a vector of vector of tuples, where each tuple is
// (row, col, time, task_id).
vector<vector<tuple<int, int, double, int>>>
SMARTSystem::convert_path_to_smart() {
    std::vector<std::vector<std::tuple<int, int, double, int>>> new_mapf_plan;
    new_mapf_plan.resize(this->num_of_drives);
    if (screen > 0) {
        std::cout << "######################################" << std::endl;
        printf("Num of agents: %d\n", this->num_of_drives);
    }

    vector<Path> raw_new_path = this->solver.solution;

    for (int i = 0; i < this->num_of_drives; i++) {
        cout << "Agent " << i << ": ";
        // The first simulation_window steps of the path are collision free
        int goal_id = 0;
        for (int t = 0; t < this->simulation_window; t++) {
            // Infer task id
            State s = raw_new_path[i][t];
            int task_id = -1;
            if (this->goal_locations[i].size() > goal_id &&
                this->goal_locations[i][goal_id].location == s.location &&
                this->goal_locations[i][goal_id].orientation == s.orientation) {
                task_id = this->goal_locations[i][goal_id].id;
                goal_id += 1;
            }
            new_mapf_plan[i].emplace_back(this->G.getRowCoordinate(s.location),
                                          this->G.getColCoordinate(s.location),
                                          static_cast<double>(t), task_id);
            if (screen > 0) {
                std::cout << "(" << this->G.getRowCoordinate(s.location) << ","
                          << this->G.getColCoordinate(s.location) << ","
                          << task_id << ")->";
                if (task_id >= 0) {
                    std::cout << "End of task " << task_id << " at step " << t
                              << " ";
                }
            }
        }
        if (screen > 0) {
            std::cout << std::endl;
        }
    }
    if (screen > 0) {
        std::cout << "######################################" << std::endl;
    }
    return new_mapf_plan;
}
#include "SMARTSystem.h"

#include <algorithm>

#include "ECBS.h"
#include "LRAStar.h"
#include "PBS.h"
#include "WHCAStar.h"
#include "common.h"
#include "helper.h"

SMARTSystem::SMARTSystem(SMARTGrid &G, MAPFSolver &solver, string task_file)
    : BasicSystem(G, solver), G(G), task_file(task_file), aisle_rt(G) {
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

    // Initialize aisle rt used for aisle path planning
    aisle_rt.map_size = G.size();
    aisle_rt.k_robust = k_robust;
    aisle_rt.window = INT_MAX;

    // Initialize aisle usage
    if (this->G.get_grid_type() == SMARTGridType::ONE_BOT_PER_AISLE) {
        for (const auto &entry : this->G.get_aisle_entries()) {
            this->aisle_usage[entry] = 0;  // Initialize to 0
        }
    }

    // std::random_device rd;
    this->gen = mt19937(this->seed);

    // Create workstation distribution
    // Number of weight must be the same as number of workstations
    assert(this->G.workstation_weights.size() == this->G.workstations.size());

    this->workstation_dist = discrete_distribution<int>(
        this->G.workstation_weights.begin(), this->G.workstation_weights.end());
    // cout << "Workstation distribution: ";
    // for (auto w : this->G.workstation_weights) {
    //     cout << w << ", ";
    // }
    // cout << endl;

    bool succ = load_tasks();
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
    vector<tuple<double, double, int>> &start_locs,
    set<int> finished_tasks_id) {
    throw std::runtime_error(
        "SMARTSystem::update_start_locations is not implemented. "
        "Please implement this function for SMARTSystem.");
    // if (start_locs.size() != this->num_of_drives) {
    //     cout << "SMARTSystem::update_start_locations: start_locs size does
    //     not "
    //             "match num_of_drives."
    //          << endl;
    // }

    // if (screen > 0) {
    //     string new_finished_tasks = "New finished tasks: ";
    //     // cout << "New finished tasks: ";
    //     for (const auto &task_id : finished_tasks_id) {
    //         // cout << task_id << " ";
    //         new_finished_tasks += std::to_string(task_id) + " ";
    //     }
    //     // cout << endl;
    //     spdlog::info(new_finished_tasks);
    // }

    // if (!this->goal_locations.empty()) {
    //     // Remove finished tasks from goal locations
    //     for (int i = 0; i < this->num_of_drives; i++) {
    //         for (int j = 0; j < (int)goal_locations[i].size(); j++) {
    //             if (finished_tasks_id.find(goal_locations[i][j].id) !=
    //                 finished_tasks_id.end()) {
    //                 goal_locations[i][j].id = -1;  // reset goal id
    //                 if (this->G.get_grid_type() ==
    //                         SMARTGridType::ONE_BOT_PER_AISLE &&
    //                     this->G.types[goal_locations[i][j].location] ==
    //                         "Endpoint") {
    //                     this->aisle_usage[this->G.aisle_entry(
    //                         goal_locations[i][j].location)] -= 1;
    //                 }
    //             }
    //         }
    //     }

    //     // Remove goals that are already finished
    //     for (int i = 0; i < this->num_of_drives; i++) {
    //         goal_locations[i].erase(
    //             remove_if(goal_locations[i].begin(), goal_locations[i].end(),
    //                       [](const Task &t) { return t.id == -1; }),
    //             goal_locations[i].end());
    //     }
    // }

    // // Set new starts
    // for (int i = 0; i < this->num_of_drives; i++) {
    //     // Obtain the starts
    //     int row = static_cast<int>(std::get<0>(start_locs[i]));
    //     int col = static_cast<int>(std::get<1>(start_locs[i]));
    //     int ori = std::get<2>(start_locs[i]);
    //     this->starts[i] = State(this->G.getCellId(row, col), 0);
    //     if (this->consider_rotation) {
    //         this->starts[i].orientation = this->G.ORI_SMART_TO_RHCR.at(ori);
    //     }
    // }
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
    throw std::runtime_error("SMARTSystem::gen_next_goal is not implemented. "
                             "Please implement this function for SMARTSystem.");
    // int next = -1;
    // if (!this->random_task && !this->tasks[agent_id].empty()) {
    //     Task next = this->tasks[agent_id].front();
    //     this->tasks[agent_id].pop_front();
    //     return next.location;  // Only take the location of the task
    // }

    // // Under w mode, alternate goal locations between workstations and
    // endpoints

    // if (this->next_goal_type[agent_id] == "w") {
    //     if (repeat_last_goal) {
    //         next = G.endpoints[rand() % (int)G.endpoints.size()];
    //         // next = this->sample_end_points();
    //     } else {
    //         next = sample_workstation();
    //         this->next_goal_type[agent_id] = "e";
    //     }
    // } else if (this->next_goal_type[agent_id] == "e") {
    //     if (repeat_last_goal) {
    //         next = sample_workstation();
    //     } else {
    //         // next = this->sample_end_points();
    //         // if (this->G.get_grid_type() == SMARTGridType::REGULAR)
    //         next = G.endpoints[rand() % (int)G.endpoints.size()];
    //         // else if (this->G.get_grid_type() ==
    //         //          SMARTGridType::ONE_BOT_PER_AISLE) {
    //         //     // Sample an endpoint from the aisle with the smallest
    //         number
    //         //     // of aisle_usage, break ties randomly
    //         //     int min_aisle_id = select_min_key_random_tie(
    //         //         this->aisle_usage, this->seed);
    //         //     // Sample an endpoint from the aisle with the smallest
    //         usage
    //         //     auto aisle = this->G.get_aisle(min_aisle_id);
    //         //     int idx = rand() % (int)aisle.size();
    //         //     auto it = aisle.begin();
    //         //     std::advance(it, idx);
    //         //     next = *it;
    //         // }
    //         this->next_goal_type[agent_id] = "w";
    //     }
    // } else {
    //     // std::cout << "error! next goal type is not w or e, but "
    //     //           << this->next_goal_type[agent_id] << std::endl;
    //     spdlog::error(
    //         "SMARTSystem::gen_next_goal: next goal type is not w or e, but
    //         {}", this->next_goal_type[agent_id]);
    //     exit(1);
    // }

    // return next;
}

void SMARTSystem::update_goal_locations() {
    throw std::runtime_error(
        "SMARTSystem::update_goal_locations is not implemented. "
        "Please implement this function for SMARTSystem.");
    // if (!this->rule_based_called)
    //     new_agents.clear();

    // // Initialize next_goal_type
    // if (this->next_goal_type.empty()) {
    //     for (int i = 0; i < num_of_drives; i++) {
    //         int start_loc = this->starts[i].location;
    //         // If start from workstation, next goal is endpoint
    //         if (G.types[start_loc] == "Workstation") {
    //             this->next_goal_type.push_back("e");
    //         }
    //         // If start from endpoint, next goal is workstation
    //         else if (G.types[start_loc] == "Endpoint") {
    //             this->next_goal_type.push_back("w");
    //         }
    //         // Otherwise, randomize the first goal
    //         else {
    //             // Randomly choose a goal location and infer its type
    //             int tmp_goal_loc =
    //                 G.task_locations[rand() %
    //                                  static_cast<int>(G.task_locations.size())];
    //             if (G.types[tmp_goal_loc] == "Workstation") {
    //                 this->next_goal_type.push_back("w");
    //             } else if (G.types[tmp_goal_loc] == "Endpoint") {
    //                 this->next_goal_type.push_back("e");
    //             } else {
    //                 std::cout << "ERROR in update_goal_locations()"
    //                           << std::endl;
    //                 std::cout << "The fiducial type at start=" << start_loc
    //                           << " should not be " << G.types[start_loc]
    //                           << std::endl;
    //                 exit(-1);
    //             }
    //             // this->next_goal_type.push_back("w");
    //         }
    //     }
    // }

    // // RHCR Algorithm
    // for (int k = 0; k < num_of_drives; k++) {
    //     int curr = starts[k].location;  // current location
    //     Task goal;                      // The last goal location
    //     if (goal_locations[k].empty()) {
    //         // goal = make_tuple(curr, 0, 0);
    //         goal = Task(curr, -1, 0, 0);
    //     } else {
    //         goal = goal_locations[k].back();
    //     }
    //     double min_timesteps = 0;
    //     int prev_loc = curr;
    //     for (const auto &g : goal_locations[k]) {
    //         min_timesteps += G.get_Manhattan_distance(g.location, prev_loc);
    //         prev_loc = g.location;
    //     }
    //     // double min_timesteps = G.get_Manhattan_distance((goal.location),
    //     // curr);
    //     while (min_timesteps <= simulation_window ||
    //            goal_locations[k].size() < 2)
    //     // The agent might finish its tasks during the next planning
    //     // horizon
    //     {
    //         // assign a new task
    //         Task next;
    //         if (G.types[goal.location] == "Endpoint" ||
    //             G.types[goal.location] == "Workstation" ||
    //             G.types[goal.location] == "Travel") {
    //             // next = make_tuple(this->gen_next_goal(k), 0, 0);
    //             next = Task(this->gen_next_goal(k), -1, 0, 0);
    //             while (next == goal) {
    //                 // next = make_tuple(
    //                 //     this->gen_next_goal(k, true), 0, 0);
    //                 next = Task(this->gen_next_goal(k, true), -1, 0, 0);
    //             }
    //             if (this->G.get_grid_type() ==
    //                     SMARTGridType::ONE_BOT_PER_AISLE &&
    //                 G.types[next.location] == "Endpoint") {
    //                 // If the next goal is an endpoint, increment the aisle
    //                 // usage for the aisle that contains this endpoint
    //                 int aisle_id = this->G.aisle_entry(next.location);
    //                 this->aisle_usage[aisle_id] += 1;
    //             }
    //         } else {
    //             std::cout << "ERROR in update_goal_function()" << std::endl;
    //             std::cout << "The fiducial type at curr=" << curr
    //                       << " should not be " << G.types[curr] << std::endl;
    //             exit(-1);
    //         }
    //         next.id = task_id;
    //         task_id += 1;  // Increment the global task ID
    //         goal_locations[k].emplace_back(next);
    //         min_timesteps +=
    //             G.get_Manhattan_distance(next.location, goal.location);
    //         goal = next;
    //     }
    // }

    // // Log the current start and goal locations in the format of () -> () ->
    // ... if (screen > 0) {
    //     // cout << "SMARTSystem::update_goal_locations: "
    //     //      << "Current start and goal locations:" << endl;
    //     spdlog::info("Current start and goal locations:");
    //     this->print_mapf_instance(this->starts, this->goal_locations);
    //     if (screen > 1) {
    //         // Check for consecutive duplicate goals for each agent
    //         for (int i = 0; i < num_of_drives; i++) {
    //             if (goal_locations[i].size() > 1) {
    //                 auto &goals = goal_locations[i];
    //                 for (size_t j = 1; j < goals.size(); j++) {
    //                     if (goals[j].location == goals[j - 1].location) {
    //                         spdlog::warn(
    //                             "Agent {} has consecutive duplicate goals at
    //                             " "location ({}, {})", i,
    //                             G.getRowCoordinate(goals[j].location),
    //                             G.getColCoordinate(goals[j].location));
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }
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
    // std::cout << "*** Simulating " << seed << " ***" << std::endl;
    spdlog::info("*** Simulating {} ***", seed);
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

        string result_message = client.call("get_location").as<string>();

        auto result_json = json::parse(result_message);
        if (!result_json["initialized"].get<bool>()) {
            // printf("Planner not initialized! Retrying\n");
            spdlog::info("Planner not initialized! Retrying");
            sleep(1);
            continue;
        }
        // auto commit_cut =
        //     result_json["robots_location"]
        //         .get<std::vector<std::tuple<double, double, int>>>();
        // auto new_finished_tasks_id =
        //     result_json["new_finished_tasks"].get<std::set<int>>();

        // Update start locations
        if (!result_json.contains("mapf_instance")) {
            spdlog::error(
                "SMARTSystem::simulate: mapf_instance not found in the JSON "
                "from server. Retrying...");
            exit(1);
        }

        if (!result_json["mapf_instance"].contains("starts") ||
            !result_json["mapf_instance"].contains("goals")) {
            spdlog::error(
                "SMARTSystem::simulate: starts or goals not found in the "
                "mapf_instance from server. Retrying...");
            exit(1);
        }
        json mapf_instance = result_json["mapf_instance"];
        this->starts = mapf_instance.at("starts").get<vector<State>>();
        // Update orientation of start location if the planner does not consider
        // rotation
        if (!this->consider_rotation) {
            for (auto &s : this->starts) {
                s.orientation = -1;
            }
        }
        this->goal_locations =
            mapf_instance.at("goals").get<vector<vector<Task>>>();
        // update_start_locations(commit_cut, new_finished_tasks_id);
        // update_goal_locations();
        solve();

        json new_plan_json = {
            // The planner is considered successful if it did not call the
            // backup planner.
            {"success", !this->rule_based_called},
            // {"success", false},
            {"plan", this->convert_path_to_smart()},
            {"congested", this->congested()},
            {"stats", this->get_curr_stats()},
            // Tmp: Pass the current MAPF instance to the backup planner if the
            // planner is not successful
            // TODO: Remove this after the task assigner is migrated to server.
            // {"mapf_instance", this->convert_mapf_instance_to_smart()},
        };

        client.call("add_plan", new_plan_json.dump());
        sleep(0.1);  // Give the server some time to process the plan
    }

    // Never end. This process will be terminated by the python script.
}

// Convert the solution of RHCR to SMART format
// The format is a vector of vector of tuples, where each tuple is
// (row, col, time, task_id).
// Orientation will be inferred by SMART (ADG). If orientatio is considered
// during planning, that should yield better path for ADG.
vector<vector<tuple<int, int, double, int>>>
SMARTSystem::convert_path_to_smart() {
    std::vector<std::vector<std::tuple<int, int, double, int>>> new_mapf_plan;

    // Planner failed, return empty plan
    if (this->rule_based_called) {
        return new_mapf_plan;
    }

    new_mapf_plan.resize(this->num_of_drives);
    if (screen > 0) {
        // std::cout << "######################################" << std::endl;
        // printf("Num of agents: %d\n", this->num_of_drives);
        spdlog::info("Plan Result:");
    }

    vector<Path> raw_new_path = this->solver.solution;

    for (int i = 0; i < this->num_of_drives; i++) {
        if (screen > 0) {
            std::cout << "Agent " << i << ": ";
        }
        // The first simulation_window steps of the path are collision free
        int goal_id = 0;
        for (int t = 0; t < this->simulation_window; t++) {
            // Infer task id
            State s = raw_new_path[i][t];
            int task_id = -1;
            Task curr_goal = this->goal_locations[i][goal_id];
            if (this->goal_locations[i].size() > goal_id &&
                curr_goal.location == s.location &&
                (curr_goal.orientation == -1 ||
                 curr_goal.orientation >= 0 &&
                     curr_goal.orientation == s.orientation)) {
                task_id = curr_goal.id;
                goal_id += 1;
            }
            new_mapf_plan[i].emplace_back(this->G.getRowCoordinate(s.location),
                                          this->G.getColCoordinate(s.location),
                                          static_cast<double>(t), task_id);
            if (screen > 0) {
                std::cout << "(" << "t=" << s.timestep << ","
                          << this->G.getRowCoordinate(s.location) << ","
                          << this->G.getColCoordinate(s.location) << ","
                          << s.orientation << "," << task_id << ")->";
                if (task_id >= 0) {
                    std::cout << "End of task " << task_id << " at step " << t
                              << "->";
                }
            }
        }
        if (screen > 0) {
            std::cout << std::endl;
        }
    }
    return new_mapf_plan;
}

json SMARTSystem::convert_mapf_instance_to_smart() {
    if (this->rule_based_called) {
        return json{
            {"starts", this->starts},
            {"goals", this->goal_locations},
        };
    } else
        return json{};
    // return json{
    //     {"starts", this->starts},
    //     {"goals", this->goal_locations},
    // };
}

bool SMARTSystem::congested() const {
    // Planner failed, do not consider congested in the planner, as it will be
    // considered in the server when backup planner is called.
    if (this->rule_based_called)
        return false;

    // Count the number of agents that are not making progress in the current
    int wait_agents = 0;
    for (const auto &path : this->solver.solution) {
        int t = 0;
        while (t < simulation_window &&
               path[timestep].location == path[timestep + t].location &&
               path[timestep].orientation == path[timestep + t].orientation &&
               !path[timestep].is_rotating)
            t++;
        if (t == simulation_window)
            wait_agents++;
    }
    // more than half of drives didn't make progress
    return wait_agents > num_of_drives / 2;
}

string SMARTSystem::get_curr_stats() const {
    json stats = {
        {"n_mapf_calls", this->n_mapf_calls},
        {"n_rule_based_calls", this->n_rule_based_calls},
    };
    return stats.dump();
}

bool SMARTSystem::load_tasks() {
    // cout << "Loading tasks from file: " << this->task_file << endl;
    spdlog::info("Loading tasks from file: {}", this->task_file);
    this->tasks = vector<list<Task>>(this->num_of_drives);
    if (this->task_file.empty() || this->task_file == "") {
        spdlog::info("No task file provided, using random tasks.");
        this->random_task = true;
        return false;  // No agent file provided, return false
    }
    try {
        this->tasks = read_task_vec(this->task_file, this->num_of_drives);
    } catch (const std::runtime_error &e) {
        // cout << "Error reading task file: " << e.what() << endl;
        spdlog::error("Error reading task file: {}", e.what());
        return false;
    }

    // Loading task is successful, we shall use the loaded tasks onward.
    this->random_task = false;
    spdlog::info("Loaded {} tasks for {} agents.", this->tasks[0].size(),
                 this->num_of_drives);

    return true;
}

void SMARTSystem::solve() {
    // Back up solvers
    this->rule_based_called = false;
    LRAStar lra(G, solver.path_planner);
    lra.simulation_window = simulation_window;
    lra.k_robust = k_robust;
    lra.gen = this->solver.gen;
    lra.screen = this->screen;

    PIBT pibt(G, solver.path_planner);
    pibt.simulation_window = simulation_window;
    pibt.k_robust = k_robust;
    pibt.gen = this->solver.gen;
    pibt.screen = this->screen;

    solver.clear();
    this->n_mapf_calls++;

    // If each aisle only serves one robot, we should "push" the start location
    // of the robot to the entry point of the aisle, and remove the first goal
    // of the robot if necessart.
    vector<vector<Task>> real_goal_locations;
    real_goal_locations.resize(num_of_drives);

    if (this->G.get_grid_type() == SMARTGridType::REGULAR) {
        // If we consider waiting time of tasks, we should ignore the first
        // goal of the agent if the agent still doing task in that goal
        // from the previous windowed MAPF run
        for (int k = 0; k < num_of_drives; k++) {
            int j = 0;
            // Agent is still doing task if the next goal is the same at the
            // start location
            if (this->is_tasking[k] &&
                starts[k].location == goal_locations[k].front().location &&
                (goal_locations[k].front().orientation == -1 ||
                 goal_locations[k].front().orientation ==
                     starts[k].orientation)) {
                j = 1;
            }
            for (; j < goal_locations[k].size(); j++) {
                real_goal_locations[k].emplace_back(goal_locations[k][j]);
            }
        }

        if (screen > 0) {
            spdlog::info("Post-processed goal locations for REGULAR:");
            this->print_mapf_instance(this->starts, real_goal_locations);
        }
    }

    else if (this->G.get_grid_type() == SMARTGridType::ONE_BOT_PER_AISLE) {
        // When the grid type is ONE_BOT_PER_AISLE, we should post-process the
        // start and goals such that:
        // 1. If the agent is still inside the aisle, we should push the
        //    start location to the entry point of the aisle, and remove the
        //    first goal of the agent if necessary. The start timestep of the
        //    agent should be updated to the distance it still needs to travel
        //    in the aisle
        // 2. For each of the remaining goals, if the goal is an endpoint
        //    (inside the aisle), we should move the goal location to the aisle
        //    entry point while set the timestep it shall wait to be the
        //    distance it needs to travel inside the aisle. By doing this,
        //    other agents cannot enter the aisle until the agent leave the
        //    aisle.

        init_aisle_paths.clear();
        aisle_paths.clear();

        for (int k = 0; k < num_of_drives; k++) {
            int j = 0;
            // Agent is still in the aisle. We need to update the start
            // location and timestep
            if (this->G.in_aisle(this->starts[k].location)) {
                // Compute the number of timesteps required by the agent to go
                // to the exit point of the aisle If the first goal is an
                // endpoint, that means the agent is still on its way to the
                // goal
                // Goal should not be empty here
                State start = this->starts[k];
                Task first_goal = goal_locations[k].front();
                int aisle_entry = this->G.aisle_entry(start.location);
                int distance_to_travel_in_aisle = 0;
                if (this->G.types[first_goal.location] == "Endpoint" &&
                    this->G.getColCoordinate(first_goal.location) ==
                        this->G.getColCoordinate(aisle_entry)) {
                    // The agent is still on its way to the first goal, which is
                    // an endpoint. It shall first travel to the first goal, and
                    // then to the aisle entry
                    if (screen > 1) {
                        spdlog::info(
                            "Agent {} is still on its way to the first goal: "
                            "({},{})",
                            k, G.getRowCoordinate(first_goal.location),
                            G.getColCoordinate(first_goal.location));
                    }
                    Path aisle_path = this->get_aisle_path(
                        start, {first_goal, Task(aisle_entry, -1, 0)},
                        this->G.get_aisle(aisle_entry), this->G.move);
                    distance_to_travel_in_aisle = aisle_path.size() - 1;

                    // Sanity check
                    if (distance_to_travel_in_aisle !=
                        G.get_Manhattan_distance(start.location,
                                                 first_goal.location) +
                            G.get_Manhattan_distance(first_goal.location,
                                                     aisle_entry)) {
                        spdlog::error(
                            "Distance to travel in aisle is not correct for "
                            "agent {}: {} != {} + {}",
                            k, distance_to_travel_in_aisle,
                            G.get_Manhattan_distance(start.location,
                                                     first_goal.location),
                            G.get_Manhattan_distance(first_goal.location,
                                                     aisle_entry));
                        throw std::runtime_error(
                            "Distance to travel in aisle is not correct");
                    }

                    // aisle_paths[first_goal.id] = aisle_path;
                    init_aisle_paths[k] = aisle_path;

                    j = 1;  // The MAPF planner shall skip the first goal
                } else {
                    if (screen > 1) {
                        spdlog::info(
                            "Agent {} is going to the aisle entry: ({},{})", k,
                            G.getRowCoordinate(aisle_entry),
                            G.getColCoordinate(aisle_entry));
                    }
                    // otherwise, the agent travel from the current
                    // location to the exit of the aisle
                    Path aisle_path = this->get_aisle_path(
                        start, {Task(aisle_entry, -1, 0)},
                        this->G.get_aisle(aisle_entry), this->G.move);

                    distance_to_travel_in_aisle = aisle_path.size() - 1;

                    // Sanity check
                    if (distance_to_travel_in_aisle !=
                        G.get_Manhattan_distance(start.location, aisle_entry)) {
                        spdlog::error(
                            "Distance to travel in aisle is not correct for "
                            "agent {}: {} != {}",
                            k, distance_to_travel_in_aisle,
                            G.get_Manhattan_distance(start.location,
                                                     aisle_entry));
                        throw std::runtime_error(
                            "Distance to travel in aisle is not correct");
                    }

                    // The agent's path shall start with the aisle path
                    init_aisle_paths[k] = aisle_path;
                }

                // The start timestep of the agent shall be the distance to the
                // exit point of the aisle, plus the current start timestep
                this->starts[k].timestep += distance_to_travel_in_aisle;

                // The start location of the agent shall be the exit point
                // of the aisle
                this->starts[k].location = aisle_entry;

                // The start location shall be "task_wait" as a hacky way to
                // make the low level planner insert wait actions at the start
                this->starts[k].is_tasking_wait = true;
            }

            // For each of the remaining goals, if the goal is an endpoint
            // (inside the aisle), we should process the goal accordingly
            for (; j < goal_locations[k].size(); j++) {
                Task real_task = goal_locations[k][j];
                if (this->G.in_aisle(real_task.location)) {
                    int aisle_entry = this->G.aisle_entry(real_task.location);

                    // The agent shall travel from the aisle entry point to the
                    // goal, then from the goal back to the aisle entry point
                    State aisle_start = State(aisle_entry, 0, -1);
                    Path aisle_path = this->get_aisle_path(
                        aisle_start, {real_task, Task(aisle_entry, -1, 0)},
                        this->G.get_aisle(aisle_entry), this->G.move);
                    int distance_to_travel_in_aisle = aisle_path.size() - 1;

                    // Sanity check
                    if (distance_to_travel_in_aisle !=
                        2 * G.get_Manhattan_distance(real_task.location,
                                                     aisle_entry)) {
                        spdlog::error(
                            "Distance to travel in aisle is not correct for "
                            "agent {}: {} != 2 * {}",
                            k, distance_to_travel_in_aisle,
                            G.get_Manhattan_distance(real_task.location,
                                                     aisle_entry));
                        throw std::runtime_error(
                            "Distance to travel in aisle is not correct");
                    }

                    aisle_paths[real_task.id] = aisle_path;

                    // The goal location should be the aisle entry point
                    // The agent should wait at the aisle entry point for the
                    // distance to travel in the aisle
                    Task shadow_task = Task(real_task);
                    shadow_task.location = aisle_entry;
                    shadow_task.task_wait_time =
                        distance_to_travel_in_aisle + real_task.task_wait_time;
                    real_goal_locations[k].emplace_back(shadow_task);
                }
                // Other tasks are added as usual.
                else {
                    real_goal_locations[k].emplace_back(goal_locations[k][j]);
                }
            }
        }

        if (screen > 0) {
            spdlog::info(
                "Post-processed goal locations for ONE_BOT_PER_AISLE:");
            this->print_mapf_instance(this->starts, real_goal_locations);

            // Print init_aisle_paths
            spdlog::info("Initial aisle paths:");
            for (const auto &pair : init_aisle_paths) {
                int agent_id = pair.first;
                const Path &aisle_path = pair.second;
                cout << "Agent " << agent_id << ": ";
                for (const auto &state : aisle_path) {
                    cout << "(" << this->G.getRowCoordinate(state.location)
                         << "," << this->G.getColCoordinate(state.location)
                         << "," << state.timestep << ") -> ";
                }
                cout << endl;
            }

            // Print aisle paths
            spdlog::info("Aisle paths:");
            for (const auto &pair : aisle_paths) {
                int task_id = pair.first;
                const Path &aisle_path = pair.second;
                cout << "Task " << task_id << ": ";
                for (const auto &state : aisle_path) {
                    cout << "(" << this->G.getRowCoordinate(state.location)
                         << "," << this->G.getColCoordinate(state.location)
                         << "," << state.timestep << ") -> ";
                }
                cout << endl;
            }
        }

    } else {
        throw std::runtime_error("Unknown grid type");
    }

    this->solve_helper(lra, pibt, real_goal_locations);

    // Stop here if the planner fails
    if (this->rule_based_called) {
        return;
    }

    if (screen > 0) {
        if (!this->validateSolution()) {
            spdlog::error(
                "Solution is NOT valid after solving the MAPF instance.");
            throw std::runtime_error(
                "Solution is NOT valid after solving the MAPF instance.");
        } else {
            spdlog::info("Solution is valid after solving the MAPF instance.");
        }
    }

    // Print path found by the solver
    if (screen > 0) {
        spdlog::info("Raw Path found by the solver:");
        for (int k = 0; k < num_of_drives; k++) {
            std::cout << "Agent " << k << ": ";
            for (const auto &state : this->solver.solution[k]) {
                std::cout << "(" << "t=" << state.timestep << ","
                          << this->G.getRowCoordinate(state.location) << ","
                          << this->G.getColCoordinate(state.location) << ","
                          << state.orientation << "," << state.is_tasking_wait
                          << ") -> ";
            }
            std::cout << std::endl;
        }
    }

    if (this->G.get_grid_type() == SMARTGridType::ONE_BOT_PER_AISLE) {
        // Here the path solved by the solver contains movement everywhere
        // except for in the aisles. We need to populate the paths with the
        // aisle paths.
        if (screen > 0)
            spdlog::info(
                "Populating paths with aisle paths for ONE_BOT_PER_AISLE");
        for (int k = 0; k < num_of_drives; k++) {
            // If the agent starts in the aisle, we should replace the init
            // aisle path to the agent's path
            if (init_aisle_paths.find(k) != init_aisle_paths.end()) {
                if (screen > 1) {
                    spdlog::info("Agent {} starts in the aisle, replacing the "
                                 "initial aisle "
                                 "path",
                                 k);
                }

                Path &path = this->solver.solution[k];
                Path &aisle_path = init_aisle_paths[k];
                int path_size = path.size();
                int aisle_path_size = aisle_path.size();
                // Replace the first aisle_path_size states with the aisle path
                int aisle_path_idx = 0;
                int state_idx = 0;
                while (state_idx < path_size &&
                       aisle_path_idx < aisle_path_size) {
                    State &state = this->solver.solution[k][state_idx];
                    State &aisle_state = aisle_path[aisle_path_idx];
                    // spdlog::info(
                    //     "Replacing state {} with aisle path state {} for
                    //     agent "
                    //     "{}",
                    //     state.location, aisle_state.location, k);
                    // // Print path
                    // if (screen > 0) {
                    //     spdlog::info("path after populating with aisle
                    //     paths:",
                    //                  k);
                    //     for (int k_ = 0; k_ < num_of_drives; k_++) {
                    //         for (const auto &state :
                    //              this->solver.solution[k_]) {
                    //             std::cout
                    //                 << "("
                    //                 <<
                    //                 this->G.getRowCoordinate(state.location)
                    //                 << ","
                    //                 <<
                    //                 this->G.getColCoordinate(state.location)
                    //                 << "," << state.timestep << ","
                    //                 << state.is_tasking_wait << ") -> ";
                    //         }
                    //         std::cout << std::endl;
                    //     }
                    // }
                    // Update the state with the aisle path state
                    // Note: we do not change the timestep
                    state.location = aisle_state.location;
                    state.orientation = aisle_state.orientation;
                    state.is_tasking_wait = false;  // No longer waiting
                    aisle_path_idx++;
                    state_idx++;
                }

                // spdlog::info(
                //     "Finished replacing the initial aisle path for agent {}",
                //     k);
                // for (const auto &state : this->solver.solution[k]) {
                //     std::cout << "(" <<
                //     this->G.getRowCoordinate(state.location)
                //               << "," <<
                //               this->G.getColCoordinate(state.location)
                //               << "," << state.timestep << ","
                //               << state.is_tasking_wait << ") -> ";
                // }
                // std::cout << std::endl;
            }

            // For each of the endpoint goals, we should replace the waiting at
            // the aisle entry point with the aisle path
            int goal_idx = 0;
            int state_idx = 0;
            while (state_idx < this->solver.solution[k].size()) {
                State &state = this->solver.solution[k][state_idx];
                if (this->G.is_aisle_entry(state.location) &&
                    state.is_tasking_wait) {
                    if (screen > 1) {
                        spdlog::info(
                            "Agent {} is at aisle entry ({},{}), t = {}, "
                            "populating with aisle path",
                            k, this->G.getRowCoordinate(state.location),
                            this->G.getColCoordinate(state.location),
                            state.timestep);
                    }

                    // Find the goal corresponding to this state, starting from
                    // goal_idx.
                    while (goal_idx < real_goal_locations[k].size() &&
                           real_goal_locations[k][goal_idx].location !=
                               state.location) {
                        goal_idx++;
                    }

                    if (screen > 1) {
                        spdlog::info(
                            "Agent {} is in aisle at state {}, t = {}, "
                            "goal {} with index {}",
                            k, state.location, state.timestep,
                            real_goal_locations[k][goal_idx].location,
                            goal_idx);
                    }

                    if (goal_idx >= real_goal_locations[k].size()) {
                        spdlog::error(
                            "Goal index {} out of bounds for agent {}",
                            goal_idx, k);
                        throw std::runtime_error(
                            "Goal index out of bounds for agent");
                    }

                    // Find the aisle path for this goal
                    int task_id = real_goal_locations[k][goal_idx].id;
                    if (aisle_paths.find(task_id) == aisle_paths.end()) {
                        spdlog::error(
                            "Aisle path for task {} not found for agent {}",
                            task_id, k);
                        throw std::runtime_error(
                            "Aisle path not found for task");
                    }

                    Path &aisle_path = aisle_paths[task_id];
                    int aisle_path_size = aisle_path.size();
                    if (aisle_path_size == 1) {
                        // A one step path. No need to replace anything.
                        State &state = this->solver.solution[k][state_idx];
                        state.is_tasking_wait = false;  // No longer waiting
                        state_idx++;
                    } else {
                        // Replace the next aisle_path_size states with the
                        // aisle path
                        // Start from 1 to skip the first state, which is the
                        // aisle entry
                        int aisle_path_idx = 1;
                        while (state_idx < this->solver.solution[k].size() &&
                               aisle_path_idx < aisle_path_size) {
                            State &state = this->solver.solution[k][state_idx];
                            State &aisle_state = aisle_path[aisle_path_idx];
                            // Update the state with the aisle path state
                            // Note: we do not change the timestep
                            state.location = aisle_state.location;
                            state.orientation = aisle_state.orientation;
                            state.is_tasking_wait = false;  // No longer waiting
                            aisle_path_idx++;
                            state_idx++;
                        }
                    }

                } else {
                    state_idx++;
                }
            }
        }
        if (screen > 0) {
            // For collision between all agents
            if (!this->validateSolution()) {
                spdlog::error("Solution is not valid after populating aisle "
                              "paths for ONE_BOT_PER_AISLE");
                throw std::runtime_error(
                    "Solution is not valid after populating aisle paths");
            } else {
                spdlog::info("Solution is valid after populating aisle paths "
                             "for ONE_BOT_PER_AISLE");
            }
        }
    }
}

Path SMARTSystem::get_aisle_path(State start, const vector<Task> &tasks,
                                 const set<int> &aisle, const int move[4]) {
    // Run a simple multi-label BFS to find path from start to tasks
    // We cannot use path_planner in solver here because the graph there has
    // the aisles as obstacles.
    Path path;
    State curr_start = State(start);
    for (const auto &task : tasks) {
        queue<int> q;
        set<int> visited;
        unordered_map<int, int> parent;
        q.push(curr_start.location);
        visited.insert(curr_start.location);
        parent[curr_start.location] = -1;
        while (!q.empty()) {
            int curr = q.front();
            q.pop();

            // Exit if goal_id is equal to the number of tasks
            if (curr == task.location) {
                Path curr_path;
                while (curr != -1) {
                    State state = State(curr, 0, -1);
                    curr_path.insert(curr_path.begin(), state);
                    curr = parent[curr];
                }

                // Remove the duplicate last state
                if (!path.empty())
                    path.erase(path.end() - 1);

                path.insert(path.end(), curr_path.begin(), curr_path.end());
                curr_start = State(task.location, 0, -1);
                break;
            }

            // Check all neighbors
            for (int i = 0; i < 4; i++) {
                int next = curr + move[i];
                if (aisle.count(next) > 0 && !visited.count(next)) {
                    visited.insert(next);
                    parent[next] = curr;
                    q.push(next);
                }
            }
        }
    }
    // should never happen
    if (path.empty()) {
        spdlog::error("No path found in get_aisle_path for start: ({},{})",
                      G.getRowCoordinate(start.location),
                      G.getColCoordinate(start.location));
        throw std::runtime_error("No path found in get_aisle_path");
    }
    return path;
}

bool SMARTSystem::validateSolution() const {
    // Check whether the paths are feasible.
    double soc = 0;
    for (int a1 = 0; a1 < num_of_drives; a1++) {
        for (int a2 = a1 + 1; a2 < num_of_drives; a2++) {
            size_t min_path_length = this->simulation_window;
            for (size_t timestep = 0; timestep < min_path_length; timestep++) {
                int loc1 = this->solver.solution[a1][timestep].location;
                int loc2 = this->solver.solution[a2][timestep].location;
                if (loc1 == loc2) {
                    cout << "Agents " << a1 << " and " << a2 << " collides at "
                         << loc1 << " at timestep " << timestep << endl;
                    return false;
                } else if (timestep < min_path_length - 1 &&
                           loc1 == this->solver.solution[a2][timestep + 1]
                                       .location &&
                           loc2 == this->solver.solution[a1][timestep + 1]
                                       .location) {
                    cout << "Agents " << a1 << " and " << a2 << " collides at ("
                         << loc1 << "-->" << loc2 << ") at timestep "
                         << timestep << endl;
                    return false;
                }
            }

            // Don't need target conflict as the agents will disappear at goal.
            // if (this->solver.solution[a1]->size() !=
            // this->solver.solution[a2]->size())
            // {
            // 	int a1_ = this->solver.solution[a1]->size() <
            // this->solver.solution[a2]->size() ? a1 : a2; 	int a2_ =
            // this->solver.solution[a1]->size() <
            // this->solver.solution[a2]->size() ? a2 : a1; 	int loc1 =
            // this->solver.solution[a1_]->back().location; 	for (size_t
            // timestep = min_path_length; timestep <
            // this->solver.solution[a2_]->size(); timestep++)
            // 	{
            // 		int loc2 =
            // this->solver.solution[a2_][timestep).location; 		if (loc1
            // == loc2)
            // 		{
            // 			cout << "Agents " << a1 << " and " << a2 << "
            // collides at
            // "
            // << loc1 << " at timestep " << timestep << endl;
            // return false; // It's at least a semi conflict
            // 		}
            // 	}
            // }
        }
    }

    return true;
}
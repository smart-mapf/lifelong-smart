#include "GreyOrangeSystem.h"

#include <algorithm>

#include "ECBS.h"
#include "LRAStar.h"
#include "PBS.h"
#include "WHCAStar.h"
#include "common.h"
#include "helper.h"

GreyOrangeSystem::GreyOrangeSystem(GreyOrangeGrid &G, MAPFSolver &solver,
                                   string task_file, string agent_file)
    : BasicSystem(G, solver),
      G(G),
      task_file(task_file),
      agent_file(agent_file) {
}

GreyOrangeSystem::~GreyOrangeSystem() {
}

void GreyOrangeSystem::initialize() {
    initialize_solvers();

    starts.resize(num_of_drives);
    goal_locations.resize(num_of_drives);
    paths.resize(num_of_drives);
    finished_tasks.resize(num_of_drives);
    waited_time.resize(num_of_drives, 0);
    is_tasking.resize(num_of_drives, false);
    next_goal_type.resize(num_of_drives, "");
    rotate_time.resize(num_of_drives, 0);
    is_rotating.resize(num_of_drives, false);

    // Initialize the goal occupied map with workstation and queue locations
    for (auto &ele : this->G.queue_locations) {
        this->goal_occupied[ele.first] = -1;
        for (auto &queue_loc : ele.second) {
            this->goal_occupied[queue_loc] = -1;
        }
    }

    bool succ = load_records();  // continue simulating from the records
    if (succ) {
        std::cout << "load_records = " << succ << ", timestep = " << timestep
                  << std::endl;
    }
    // exit(-1);

    if (!succ) {
        timestep = 0;
        // Load agent start locations from file, if given
        succ = load_agents();
        if (!succ) {
            cout << "Randomly generating initial start locations" << endl;
            initialize_start_locations();
        }

        // Load task locations from file, if given
        succ = load_tasks();
        if (!succ) {
            cout << "Randomly generating initial goal locations" << endl;
            initialize_goal_locations();
        }
    }

    // std::random_device rd;
    this->gen = mt19937(this->seed);
}

bool GreyOrangeSystem::load_agents() {
    cout << "Loading agents from file: " << this->agent_file << endl;
    if (this->agent_file.empty() || this->agent_file == "") {
        return false;  // No agent file provided, return false
    }

    vector<tuple<int, int>> agent_starts;
    try {
        agent_starts = read_start_vec(this->agent_file, this->num_of_drives);
    } catch (const std::runtime_error &e) {
        cout << "Error reading agent file: " << e.what() << endl;
        return false;
    }

    for (int k = 0; k < this->num_of_drives; k++) {
        int loc = std::get<0>(agent_starts[k]);
        int ori = std::get<1>(agent_starts[k]);
        if (ori == -1)
            ori = rand() % 4;  // Randomly choose orientation if not given
        starts[k] = State(loc, 0, ori);
        paths[k].emplace_back(starts[k]);
        finished_tasks[k].emplace_back(Task(loc, ori, 0, 0, k, 0));
    }
    return true;
}

bool GreyOrangeSystem::load_tasks() {
    cout << "Loading tasks from file: " << this->task_file << endl;
    this->tasks = vector<list<Task>>(this->num_of_drives);
    if (this->task_file.empty() || this->task_file == "") {
        this->random_task = true;
        return false;  // No agent file provided, return false
    }
    try {
        this->tasks = read_task_vec(this->task_file, this->num_of_drives);
    } catch (const std::runtime_error &e) {
        cout << "Error reading task file: " << e.what() << endl;
        return false;
    }

    // Loading task is successful, we shall use the loaded tasks onward.
    this->random_task = false;

    // Initialize the goal locations
    for (int k = 0; k < this->num_of_drives; k++) {
        auto goals = this->gen_next_goal(k);
        for (auto goal : goals) {
            this->goal_locations[k].emplace_back(goal);
        }
        if (this->screen > 0) {
            cout << "Agent " << k << ": Next goal " << goal_locations[k].front()
                 << endl;
        }
    }

    // for (int k = 0; k < this->num_of_drives; k++) {
    //     if (!this->random_task && !this->tasks[k].empty()) {
    //         Task goal = Task(this->tasks[k].front());
    //         this->goal_locations[k].emplace_back(goal);
    //         // Set next goal type based on the first task
    //         this->next_goal_type[k] =
    //             G.types[goal.location] == "Endpoint" ? "w" : "g";
    //         this->tasks[k].pop_front();
    //     }
    //     // No given task is available, let the agents go to a random parking
    //     // locations. If there are no parking locations, generate a random
    //     goal. else {
    //         if (this->G.parking_locations.size() > 0) {
    //             int idx = rand() % (int)this->G.parking_locations.size();
    //             int goal = this->G.parking_locations[idx];
    //             this->goal_locations[k].emplace_back(
    //                 Task(goal, 0, 0, 0, k, -1, true));
    //         } else {
    //             if (this->G.types[starts[k].location] == "Endpoint") {
    //                 this->next_goal_type[k] = "g";
    //             } else {
    //                 this->next_goal_type[k] = "w";
    //             }
    //             auto goal = this->gen_next_goal(k);
    //             this->goal_locations[k].emplace_back(goal);
    //         }
    //     }
    //     if (this->screen > 0) {
    //         cout << "Agent " << k << ": Next goal " <<
    //         goal_locations[k].front()
    //              << endl;
    //     }
    // }

    return true;
}

void GreyOrangeSystem::initialize_start_locations() {
    // Choose random start locations
    // Any non-obstacle locations can be start locations
    // Start locations should be unique
    for (int k = 0; k < num_of_drives; k++) {
        int orientation = -1;
        if (consider_rotation) {
            orientation = rand() % 4;
        }
        starts[k] = State(G.agent_home_locations[k], 0, orientation);
        paths[k].emplace_back(starts[k]);
        finished_tasks[k].emplace_back(
            Task(G.agent_home_locations[k], -1, 0, 0, k, 0));
    }
}

void GreyOrangeSystem::update_start_locations(int t) {
    // std::cout << "in update start location, timestep =
    // "<<timestep<<std::endl;
    // cout << "In update start locations " << endl;
    for (int k = 0; k < num_of_drives; k++) {
        // 1. Set start timestep for each agent to the number of timesteps it
        // needs to wait
        // 2. Insert proper number of wait actions at the start of the path
        // (updatePath in SIPP/Astar)

        // The agent is still doing task in the previous goal. Let it finish it.
        if (this->is_tasking[k]) {
            int start_timestep =
                goal_locations[k].front().task_wait_time - this->waited_time[k];
            if (screen > 0) {
                cout << "Tasking Agent " << k << ": start_timestep = "
                     << goal_locations[k].front().task_wait_time << " - "
                     << this->waited_time[k]
                     << ", start location = " << paths[k][timestep].location
                     << endl;
            }
            // assert(start_timestep > 0);
            if (start_timestep < 0) {
                cout << "Error while task waiting: start_timestep <= 0" << endl;
                exit(0);
            }

            starts[k] = State(paths[k][timestep].location, start_timestep,
                              paths[k][timestep].orientation, true,
                              paths[k][timestep].is_rotating);
        } else if (this->is_rotating[k]) {
            // The agent is still rotating in the previous goal. Let it finish
            // it.
            int start_timestep = this->rotation_time - this->rotate_time[k];
            if (screen > 0) {
                cout << "Rotating Agent " << k
                     << ": start_timestep = " << start_timestep
                     << ", start location = " << paths[k][timestep].location
                     << endl;
            }
            if (start_timestep < 0) {
                cout << "Error while rotating: start_timestep <= 0" << endl;
                exit(0);
            }

            starts[k] = State(paths[k][timestep].location, start_timestep,
                              paths[k][timestep].orientation, false, true);
        } else {
            starts[k] = State(paths[k][timestep].location, 0,
                              paths[k][timestep].orientation,
                              paths[k][timestep].is_tasking_wait,
                              paths[k][timestep].is_rotating);
        }
    }
}

void GreyOrangeSystem::initialize_goal_locations() {
    cout << "Initializing goal locations" << endl;

    // Assert RHCR
    if (hold_endpoints || useDummyPaths) {
        cout << "GreyOrange system must use RHCR with `hold_endpoints` and "
                "`useDummyPaths` being False"
             << endl;
        exit(0);
    }

    for (int k = 0; k < num_of_drives; k++) {
        int start_loc = this->starts[k].location;
        int goal;
        // If start from workstation, sample the next goal as an endpoint and
        // set `next_goal_type` to "w", because the next sampled goal will be a
        // workstation.
        if (std::find(G.workstations.begin(), G.workstations.end(),
                      start_loc) != G.workstations.end()) {
            this->next_goal_type[k] = "w";
            goal = G.endpoints[rand() % (int)G.endpoints.size()];
        }
        // Otherwise, next goal is workstation
        else {
            goal = sample_workstation();  // Sample a workstation
            this->next_goal_type[k] = "g";
        }
        int ori =
            this->consider_rotation ? rand() % 4 : -1;  // Random orientation
        goal_locations[k].emplace_back(Task(goal, ori, 5));
    }
}

int GreyOrangeSystem::sample_workstation() {
    // Sample a workstation based the given weight
    // int idx = this->workstation_dist(this->gen);
    int idx = rand() % (int)G.workstations.size();

    return this->G.workstations[idx];
}

vector<Task> GreyOrangeSystem::gen_next_goal(int agent_id) {
    vector<Task> new_goals;  // Next goals
    // Task next;

    // Queue mechanism
    bool queuing = false;
    // Task queue_goal;

    // There are still tasks in the task list
    if (!this->random_task && !this->tasks[agent_id].empty()) {
        Task next = this->tasks[agent_id].front();

        if (this->queue_mechanism) {
            queuing = this->update_queue_goal_locations(agent_id, next.location,
                                                        new_goals);
        }

        // Update next goal type
        if (!queuing) {
            new_goals.push_back(next);
            this->next_goal_type[agent_id] =
                G.types[next.location] == "Endpoint" ? "w" : "g";
            // if (!queuing)
            this->tasks[agent_id].pop_front();
        }
    }
    // There is no tasks in task list but we are using given task, and parking
    // locations are available. Send agents to parking locations
    else if (!this->random_task && this->tasks[agent_id].empty() &&
             this->G.parking_locations.size() > 0) {
        int idx = rand() % (int)this->G.parking_locations.size();
        int goal = this->G.parking_locations[idx];
        int ori =
            this->consider_rotation ? rand() % 4 : -1;  // Random orientation
        Task next = Task(goal, ori, 0, 0, agent_id, -1, true);
        new_goals.push_back(next);
    }
    // Generating random tasks
    else {
        // This is the first (random) goal. Determine the goal type based on
        // the start location
        if (this->next_goal_type[agent_id] == "") {
            if (this->G.types[starts[agent_id].location] == "Endpoint") {
                this->next_goal_type[agent_id] = "w";
            } else {
                this->next_goal_type[agent_id] = "g";
            }
        }

        // Generate a new goal
        int next_loc;
        if (this->next_goal_type[agent_id] == "w") {
            next_loc = sample_workstation();

            if (this->queue_mechanism) {
                this->update_queue_goal_locations(agent_id, next_loc,
                                                  new_goals);
            }
            this->next_goal_type[agent_id] = "g";

        } else if (this->next_goal_type[agent_id] == "g") {
            // next = this->sample_end_points();
            next_loc = G.endpoints[rand() % (int)G.endpoints.size()];
            this->next_goal_type[agent_id] = "w";
        } else {
            std::cout << "error! next goal type is not w or g, but "
                      << this->next_goal_type[agent_id] << std::endl;
            exit(1);
        }

        int wait_time = 5;  // TODO: Fancier wait time
        Task next = Task(next_loc, -1, wait_time, 0);
        new_goals.push_back(next);
    }

    // return next;
    return new_goals;
}

void GreyOrangeSystem::update_goal_locations(int t) {
    if (!this->rule_based_called)
        new_agents.clear();
    // RHCR Algorithm
    // First determine the goal of the agents in the queue
    // Since agents in `queue_aborted_agt` are naturally sorted from their
    // distance to the workstation, we should be sending the agent in the
    // "front" of the queue to the workstation first.
    vector<int> agents;
    for (auto &ag : this->queue_aborted_agt) {
        agents.push_back(ag);
    }
    // Then rest of the agents
    // vector<int> rest_of_agents;
    for (int k = 0; k < num_of_drives; k++) {
        if (std::find(agents.begin(), agents.end(), k) == agents.end()) {
            // rest_of_agents.push_back(k);
            agents.push_back(k);
        }
    }

    // Sort rest of the agents based on their heuristic distance to the next
    // intended goal.
    // TODO: For now it only works if the tasks are given, make it general in
    // the future.
    // std::sort(rest_of_agents.begin(), rest_of_agents.end(), [&](int a, int b)
    // {
    //     double cost_a, cost_b;
    //     if (goal_locations[a].empty()) {
    //         if (this->tasks[a].empty())
    //             cost_a = INT_MAX;
    //         else {
    //             cost_a = this->G.heuristics[this->tasks[a].front().location]
    //                                        [starts[a].location];
    //         }
    //     } else {
    //         cost_a = this->G.heuristics[goal_locations[a].front().location]
    //                                    [starts[a].location];
    //     }

    //     if (goal_locations[b].empty()) {
    //         if (this->tasks[b].empty())
    //             cost_b = INT_MAX;
    //         else {
    //             cost_b = this->G.heuristics[this->tasks[b].front().location]
    //                                        [starts[b].location];
    //         }
    //     } else {
    //         cost_b = this->G.heuristics[goal_locations[b].front().location]
    //                                    [starts[b].location];
    //     }

    //     return cost_a < cost_b;
    // });
    // agents.insert(agents.end(), rest_of_agents.begin(),
    // rest_of_agents.end());

    // for (int k = 0; k < num_of_drives; k++) {
    for (auto &k : agents) {
        int curr = paths[k][timestep].location;  // current location
        Task goal;                               // The last goal location
        if (goal_locations[k].empty()) {
            // goal = make_tuple(curr, 0, 0);
            goal = Task(curr, -1, 0, 0);
        } else {
            goal = goal_locations[k].back();
        }

        // Compute the minimum number of timesteps used to reach all the goals.
        double min_timesteps = 0;
        if (!goal_locations[k].empty()) {
            // From current location to the first goal, then wait.
            // Decrement the already waited time from the first goal.
            min_timesteps += G.get_Manhattan_distance(
                                 goal_locations[k].front().location, curr) +
                             goal_locations[k].front().task_wait_time -
                             this->waited_time[k];
            for (int g = 0; g < (int)goal_locations[k].size() - 1; g++) {
                // From the g-th to the (g+1)-th goal, then wait.
                min_timesteps += G.get_Manhattan_distance(
                                     goal_locations[k][g].location,
                                     goal_locations[k][g + 1].location) +
                                 goal_locations[k][g + 1].task_wait_time;
            }
        }

        // We should add goal until there are at least 2 goals for the agent.
        // This is because if the agent has arrived at the only goal of it and
        // has started doing task on it, the MAPF planner will ignore that
        // goal. The MAPF planner will not plan any thing which triggers error.
        // So a hacky way to solve this problem is adding another goal, but it
        // is not very efficient because we are essentially planning things
        // that are not necessary.
        // We also keep adding the next goal if it is the same as the last goal
        while (min_timesteps <= simulation_window ||
               goal_locations[k].size() < 2 ||
               (!this->tasks[k].empty() &&
                this->tasks[k].front().location == goal.location &&
                this->tasks[k].front().orientation == goal.orientation))
        // The agent might finish its tasks during the next planning horizon
        {
            // assign a new task
            vector<Task> next_goals = this->gen_next_goal(k);
            // if (G.types[goal.location] == "Endpoint" ||
            //     G.types[goal.location] == "Workstation") {
            //     next = this->gen_next_goal(k);
            //     if (this->screen > 0) {
            //         cout << "Agent " << k << ": Next goal " << next << endl;
            //     }
            // } else {
            //     std::cout << "ERROR in update_goal_function()" << std::endl;
            //     std::cout << "The fiducial type should not be " <<
            //     G.types[curr]
            //               << std::endl;
            //     exit(-1);
            // }
            for (auto &next : next_goals) {
                goal_locations[k].emplace_back(next);
                // Incorporate task wait time in expected `min_timesteps`
                min_timesteps +=
                    G.get_Manhattan_distance(next.location, goal.location) +
                    next.task_wait_time;
                goal = next;
            }
        }
    }
}

bool GreyOrangeSystem::update_queue_goal_locations(int k, int goal_loc,
                                                   vector<Task> &new_goals) {
    // cout << "In update_queue_goal_locations" << endl;
    // // Print the content of goal_occupied
    // if (screen > 0) {
    //     cout << "Occupied locations: ";
    //     for (auto &ele : this->goal_occupied) {
    //         cout << ele.first << ": " << ele.second << ", ";
    //     }
    //     cout << endl;
    // }
    // Update the goal location of agent k with the intended goal
    // If the goal is a workstation, and another agent is going to it, then we
    // send agent k to the queue loc of the workstation
    if (this->G.types[goal_loc] == "Workstation") {
        // Check if the goal is occupied by another agent
        if (this->goal_occupied[goal_loc] != -1) {
            // The goal is occupied, send the agent to the first queue location
            // that is not occupied
            for (auto &queue_loc : this->G.queue_locations[goal_loc]) {
                // Edge case: The agent is already queuing. No need to queue
                // again.
                if (this->goal_occupied[queue_loc] == k) {
                    if (screen > 0) {
                        cout << "Agent " << k << ": Next goal " << goal_loc
                             << " is occupied by "
                             << this->goal_occupied[goal_loc]
                             << ", already in queue location " << queue_loc
                             << endl;
                    }
                    return true;
                }

                // Edge case: the current goal is the same as the previous
                // goal, and the agent is not queuing. It does not need to
                // queue here either.
                if (!goal_locations[k].empty()) {
                    auto prev_goal = goal_locations[k].back();
                    // Location are the same is good enough
                    if (prev_goal.location == goal_loc) {
                        return false;
                    }
                }

                // The queue location is not occupied, set it as occupied
                if (this->goal_occupied[queue_loc] == -1) {
                    this->goal_occupied[queue_loc] = k;
                    // Set the wait time as infinite (until the end of the sim)
                    // The waiting will be aborted once the workstation is free
                    // Caveat: the wait time is set to be the simulation time so
                    // that it will wait until the end of the simulation (which,
                    // technically is "forever")
                    Task queue_goal =
                        Task(queue_loc, -1,
                             this->simulation_time - this->timestep +
                                 this->simulation_window);
                    if (screen > 0) {
                        cout << "Agent " << k << ": Next goal " << goal_loc
                             << " is occupied by "
                             << this->goal_occupied[goal_loc]
                             << ", send to queue location " << queue_loc
                             << endl;
                    }
                    new_goals.push_back(queue_goal);

                    // Hacky: Add a dummy random goal. Our implementation of
                    // SIPP requires at least two goals for each agent. We add
                    // a dummy random goal here to let the planner run. The
                    // agent will actually never go to this dummy goal because
                    // it is expected to wait "forever" at the queue location.
                    // Once the workstation is free, the queue location and the
                    // dummy goal will be removed from the goal list.
                    int dummy_goal =
                        this->G
                            .endpoints[rand() % (int)this->G.endpoints.size()];

                    new_goals.push_back(Task(dummy_goal, 0, 0, 0, k, -1, true));
                    return true;
                    // return make_tuple(true, queue_goal);
                }
            }
            // No queue location is free, go to a random parking location, if
            // any. Otherwise, go to a random free point
            if (this->G.parking_locations.size() > 0) {
                int idx = rand() % (int)this->G.parking_locations.size();
                int goal = this->G.parking_locations[idx];
                if (screen > 0) {
                    cout << "Agent " << k << ": Next goal " << goal_loc
                         << " is occupied by " << this->goal_occupied[goal_loc]
                         << ", send to parking location " << goal << endl;
                }
                new_goals.push_back(Task(goal, 0, 0, 0, k, -1, true));
                return true;
                // return make_tuple(true, Task(goal, 0, 0, 0, k, -1, true));
            } else {
                // No queue location is free, go to a random free point
                int goal =
                    this->G.endpoints[rand() % (int)this->G.endpoints.size()];
                if (screen > 0) {
                    cout << "Agent " << k << ": Next goal " << goal_loc
                         << " is occupied by " << this->goal_occupied[goal_loc]
                         << ", send to random free point " << goal << endl;
                }
                new_goals.push_back(Task(goal, 0, 0, 0, k, -1, true));
                return true;
                // return make_tuple(true, Task(goal, 0, 0, 0, k, -1, true));
            }
        } else {
            // The goal is not occupied, set it as occupied
            if (screen > 0) {
                cout << "Agent " << k << ": Next goal " << goal_loc
                     << " is not occupied" << endl;
            }
            this->goal_occupied[goal_loc] = k;
            return false;
            // return make_tuple(false, Task(goal_loc, -1, 0, 0, k, -1, false));
        }
    } else {
        if (screen > 0) {
            cout << "Agent " << k << ": Next goal " << goal_loc
                 << " is not a workstation" << endl;
        }
    }
    return false;
    // return make_tuple(false, Task(goal_loc, -1, 0, 0, k, -1, false));
}

tuple<vector<double>, double, double>
GreyOrangeSystem::edge_pair_usage_mean_std(vector<vector<double>> &edge_usage) {
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
GreyOrangeSystem::convert_edge_usage(vector<vector<double>> &edge_usage) {
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

json GreyOrangeSystem::summarizeResult() {
    // Compute objective
    double throughput = (double)this->num_of_tasks / this->simulation_time;

    // Compute measures:
    // 1. Variance of tile usage
    // 2. Average number of waiting agents at each vertex
    // 3. Average distance of the finished tasks
    // 4. Average number of rotations over the agents. If not considering
    //    rotations, this counts the number of shadow rotations
    // 5. Average number of "reversed" action (not going in the direction
    //    suggested by highway) in each timestep, weighted by absolute value of
    //    the difference of edges both directions
    // 6. Variance of edge usage weighted by absolute value of the difference
    //    of edges both directions
    // and more...
    int n_vertices = this->G.rows * this->G.cols;
    std::vector<double> tile_usage(n_vertices, 0.0);
    // Adj matrix: [i,j] stores edge usage of edge from node_i to node_j
    // std::vector<vector<double>> edge_usage(
    //     n_vertices, std::vector<double>(n_vertices, 0.0));
    // std::vector<double> num_wait(n_vertices, 0.0);

    std::vector<double> cr_usage_matrix(n_vertices, 0.0);
    std::vector<double> ccr_usage_matrix(n_vertices, 0.0);
    std::vector<double> vertex_wait_matrix(n_vertices, 0.0);
    // Note: the order in edge_usage_matrix is different from RHCR. Here we
    // have right, down, left, up, while in RHCR we have right up, left, down
    std::vector<double> edge_usage_matrix(n_vertices * 4, 0.0);

    std::vector<double> finished_task_len;
    std::vector<double> path_inefficiency;
    // std::vector<double> num_rev_action(this->simulation_time, 0.0);
    std::vector<Path> no_wait_paths(this->paths.size());
    for (int k = 0; k < num_of_drives; k++) {
        int path_length = this->paths[k].size();
        // no_wait_paths[k] = Path();
        for (int j = 0; j < path_length; j++) {
            State s = this->paths[k][j];

            // Record the path without wait actions
            if (j < path_length - 1) {
                State n_s = this->paths[k][j + 1];
                if (s.location != n_s.location &&
                    s.orientation != n_s.orientation) {
                    no_wait_paths[k].emplace_back(s);
                }
            }

            // Count tile usage
            tile_usage[s.location] += 1.0;

            // Infer the action and record the corresponding action usage
            // The planned path might go beyond the simulation window.
            if (j < path_length - 1 && s.timestep < this->simulation_time) {
                State next_s = this->paths[k][j + 1];
                // wait action
                if (s.location == next_s.location &&
                    s.orientation == next_s.orientation) {
                    if (s.is_tasking_wait) {
                        // TODO: Task wait action
                    } else if (s.is_rotating) {
                        // TODO: Rotation wait action
                    } else if (this->G.types[s.location] == "Queuepoint") {
                        // TODO: Wait action at queue point
                    } else {
                        // Normal wait action
                        vertex_wait_matrix[s.location] += 1.0;
                    }
                }
                // Rotation
                else if (s.location == next_s.location &&
                         s.orientation != next_s.orientation) {
                    // Infer CR or CCR rotation
                    if ((s.orientation + 1) % 4 == next_s.orientation) {
                        cr_usage_matrix[s.location] += this->rotation_time;
                    } else {
                        ccr_usage_matrix[s.location] += this->rotation_time;
                    }
                }
                // Move action
                else {
                    int dir = this->G.get_direction_output(s.location,
                                                           next_s.location);
                    if (dir < 0) {
                        cout << s << " " << next_s << endl;
                    } else
                        edge_usage_matrix[s.location * 4 + dir] += 1.0;
                }

                // // Edge weight from s_t to s_t+1
                // double t_to_t_p_1 = this->G.get_weight(
                //     s.location, next_s.location);
                // // Edge weight from s_t+1 to s_t
                // double t_p_1_to_t = this->G.get_weight(
                //     next_s.location, s.location);

                // Increment edge usage.
                // This DOES include number of wait action at each vertex (the
                // diagnal of the matrix), but they will be ignored while
                // calculating the mean and std.
                // edge_usage[s.location][next_s.location] += 1.0;

                // See if the action follows the edge direction "suggested" by
                // the highway system.
                // If the edge weight that the agent is traversing through is
                // larger than the direction in reverse, then the agent is NOT
                // following the suggested direction.
                // if (s.timestep < this->simulation_time &&
                //     s.location != next_s.location && t_to_t_p_1 > t_p_1_to_t)
                // {
                //     num_rev_action[s.timestep] += std::abs(t_to_t_p_1 -
                //     t_p_1_to_t);
                // }
            }
        }

        int prev_t = 0;
        Task prev_task = this->finished_tasks[k].front();
        for (auto task : this->finished_tasks[k]) {
            if (task.finish_t != 0 && !task.is_parking) {
                // Compute the ideal finished task length
                double ideal_task_len =
                    this->G.heuristics[task.location][prev_task.location];

                // path inefficiency = (actual path length - waiting) / ideal
                // path
                int actual_task_len = task.finish_t - prev_task.finish_t;
                cout << "Agent " << k << ", "
                     << "Prev task t = " << prev_task.finish_t << ", "
                     << "Task t = " << task.finish_t << ", " << endl;
                // Exclude the wait time at the goal
                actual_task_len -= task.task_wait_time;
                // Look at the path segment between the two tasks, exclude wait
                // actions at the queue points
                for (int t = prev_task.finish_t; t < task.finish_t; t++) {
                    State s = this->paths[k][t];
                    if (t + 1 < this->simulation_time) {
                        State next_s = this->paths[k][t + 1];
                        if (this->G.types[s.location] == "Queuepoint" &&
                            s.location == next_s.location &&
                            s.orientation == next_s.orientation &&
                            !s.is_rotating && !s.is_tasking_wait) {
                            actual_task_len--;
                        }
                        // HACKY: In the above if block, we are removing one
                        // more wait action that is supposed to be rotation at
                        // the queue point. We add it back here.
                        if (this->G.types[s.location] == "Queuepoint" &&
                            s.location == next_s.location &&
                            s.orientation != next_s.orientation) {
                            actual_task_len++;
                        }
                    }
                }

                if (ideal_task_len == 0) {
                    path_inefficiency.push_back(1);
                } else {
                    path_inefficiency.push_back((double)actual_task_len /
                                                ideal_task_len);
                }

                cout << "Agent " << k << ", "
                     << "Task " << task.location << ": "
                     << "actual task length = " << actual_task_len
                     << ", ideal task length = " << ideal_task_len
                     << ", path inefficiency = " << path_inefficiency.back()
                     << endl;
                if (path_inefficiency.back() < 1) {
                    // TODO: Resolve the warning
                    cout << "WARNING: path inefficiency < 1" << endl;
                }

                prev_task = task;

                // Compute the finished task length
                int curr_t = task.finish_t;
                Path p = this->paths[k];

                // Calculate length of the path associated with this task
                double task_path_len = 0.0;
                for (int t = prev_t; t < curr_t - 1; t++) {
                    if (p[t].location != p[t + 1].location) {
                        task_path_len += 1.0;
                    }
                }
                finished_task_len.push_back(task_path_len);
                prev_t = curr_t;
            }
        }
    }

    std::vector<double> num_rotations(num_of_drives, 0.0);
    for (int k = 0; k < this->num_of_drives; k++) {
        int path_length = this->paths[k].size();
        for (int j = 0; j < path_length; j++) {
            // While not considering rotations, count the number of shadow
            // rotations
            if (!this->consider_rotation) {
                if (j > 0 && j < path_length - 1) {
                    State prev_state = this->paths[k][j - 1];
                    State next_state = this->paths[k][j + 1];
                    int prev_row =
                        this->G.getRowCoordinate(prev_state.location);
                    int next_row =
                        this->G.getRowCoordinate(next_state.location);
                    int prev_col =
                        this->G.getColCoordinate(prev_state.location);
                    int next_col =
                        this->G.getColCoordinate(next_state.location);
                    // Count shadow rotations
                    if (prev_state.location == next_state.location) {
                        num_rotations[k] += 2;
                    } else if (abs(prev_row - next_row) == 1 &&
                               abs(prev_col - next_col) == 1) {
                        num_rotations[k] += 1;
                    }
                }
            }
            // Count actual rotations
            else {
                if (j < path_length - 1) {
                    State s = this->paths[k][j];
                    // State next_s = this->paths[k][j + 1];
                    if (s.is_rotating) {
                        num_rotations[k] += 1.0;
                    }
                }
            }
        }
    }

    // // Longest common sub-path
    // vector<int> subpath =
    //     helper::longest_common_subpath(this->paths, this->simulation_time);

    // Post process data
    // Normalize tile usage s.t. they sum to 1
    double tile_usage_sum = helper::sum(tile_usage);
    helper::divide(tile_usage, tile_usage_sum);

    double tile_usage_mean, tile_usage_std;
    // double edge_pair_usage_mean, edge_pair_usage_std;
    // vector<double> edge_pair_usage;
    double num_wait_mean, num_wait_std;
    double finished_len_mean, finished_len_std;
    double num_rotations_mean, num_rotations_std;
    // double num_rev_action_mean, num_rev_action_std;
    double avg_task_len = this->G.get_avg_task_len(this->G.heuristics);
    // vector<vector<vector<double>>> edge_usage_matrix;
    // vector<vector<double>> vertex_wait_matrix;

    std::tie(tile_usage_mean, tile_usage_std) = helper::mean_std(tile_usage);
    // std::tie(edge_pair_usage, edge_pair_usage_mean, edge_pair_usage_std) =
    // edge_pair_usage_mean_std(edge_usage);
    std::tie(num_wait_mean, num_wait_std) =
        helper::mean_std(vertex_wait_matrix);
    std::tie(finished_len_mean, finished_len_std) =
        helper::mean_std(finished_task_len);
    std::tie(num_rotations_mean, num_rotations_std) =
        helper::mean_std(num_rotations);
    // std::tie(num_rev_action_mean, num_rev_action_std) =
    // helper::mean_std(num_rev_action);
    // std::tie(edge_usage_matrix, vertex_wait_matrix) =
    //     convert_edge_usage(edge_usage);

    // Log some of the results
    std::cout << std::endl;
    std::cout << "Throughput: " << throughput << std::endl;
    std::cout << "Std of tile usage: " << tile_usage_std << std::endl;
    // std::cout << "Std of edge pair usage: " << edge_pair_usage_std <<
    // std::endl;
    std::cout << "Average wait at each timestep: " << num_wait_mean
              << std::endl;
    std::cout << "Average path length of each finished task: "
              << finished_len_mean << std::endl;
    std::cout << "Average path length of each task: " << avg_task_len
              << std::endl;
    std::cout << "Average number of turns: " << num_rotations_mean << std::endl;
    // std::cout << "Average number of reversed actions in highway: " <<
    // num_rev_action_mean << std::endl;
    // std::cout << "Length of longest common path: " << subpath.size()
    //           << std::endl;

    update_start_locations(0);
    std::cout << std::endl << "Done!" << std::endl;
    save_results();

    // Create the result json object
    json result;
    result = {{"throughput", throughput},
              {"tile_usage", tile_usage},
              // {"edge_pair_usage", edge_pair_usage},
              {"edge_usage_matrix", edge_usage_matrix},
              {"vertex_wait_matrix", vertex_wait_matrix},
              {"cr_usage_matrix", cr_usage_matrix},
              {"ccr_usage_matrix", ccr_usage_matrix},
              //   {"num_wait", num_wait},
              {"num_rotations", num_rotations},
              // {"num_rev_action", num_rev_action},
              {"finished_task_len", finished_task_len},
              {"path_inefficiency", path_inefficiency},
              {"tile_usage_mean", tile_usage_mean},
              {"tile_usage_std", tile_usage_std},
              // {"edge_pair_usage_mean", edge_pair_usage_mean},
              // {"edge_pair_usage_std", edge_pair_usage_std},
              {"num_wait_mean", num_wait_mean},
              {"num_wait_std", num_wait_std},
              {"finished_len_mean", finished_len_mean},
              {"finished_len_std", finished_len_std},
              {"num_rotations_mean", num_rotations_mean},
              {"num_rotations_std", num_rotations_std},
              // {"num_rev_action_mean", num_rev_action_mean},
              // {"num_rev_action_std", num_rev_action_std},
              // {"tasks_finished_timestep", tasks_finished_timestep},
              {"avg_task_len", avg_task_len},
              // {"congested", congested_sim},
              //   {"longest_common_path", subpath},
              {"stop_at_timestep", this->timestep},
              {"n_mapf_calls", this->n_mapf_calls},
              //   {"finished_tasks", this->finished_tasks},
              {"n_rule_based_calls", this->n_rule_based_calls}};
    return result;
}

json GreyOrangeSystem::summarizeCurrResult(int summarize_interval) {
    json all_paths = json::array();
    json full_paths = json::array();
    json future_paths = json::array();
    for (int k = 0; k < num_of_drives; k++) {
        int path_length = this->paths[k].size();
        if (summarize_interval > timestep ||
            summarize_interval + 1 > path_length) {
            std::cout << "warning: summarize interval [" << summarize_interval
                      << "],path length [" << path_length << "]!" << std::endl;
            std::cout << "warning: timestep [" << timestep << "]!" << std::endl;
            summarize_interval = min(path_length - 1, timestep);
        }
        json agent_path = json::array();
        for (int j = timestep - summarize_interval; j <= timestep; j++) {
            State s = this->paths[k][j];
            agent_path.push_back(s.location);
        }
        all_paths.push_back(agent_path);

        json full_path = json::array();
        for (int j = 0; j < path_length; j++) {
            State s = this->paths[k][j];
            full_path.push_back(s.location);
        }
        full_paths.push_back(full_path);

        json future_path = json::array();
        for (int j = timestep; j < path_length; j++) {
            State s = this->paths[k][j];
            future_path.push_back(s.location);
        }
        future_paths.push_back(future_path);
    }

    json goal_locs = json::array();
    for (int i = 0; i < this->goal_locations.size(); ++i) {
        int goal_loc = this->goal_locations[i][0].location;
        goal_locs.push_back(goal_loc);
    }

    json full_goal_locs = json::array();
    for (int i = 0; i < this->goal_locations.size(); ++i) {
        json full_agent_locs = json::array();
        for (int j = 0; j < this->goal_locations[i].size(); ++j) {
            int goal_loc = this->goal_locations[i][j].location;
            full_agent_locs.push_back(goal_loc);
        }
        full_goal_locs.push_back(full_agent_locs);
    }

    json result;
    result["all_paths"] = all_paths;
    result["full_paths"] = full_paths;
    result["future_paths"] = future_paths;
    result["goal_locs"] = goal_locs;
    result["full_goal_locs"] = full_goal_locs;
    result["num_tasks_finished"] = this->num_of_tasks;
    result["done"] = this->timestep >= this->total_sim_time;
    result["summarize_time"] = summarize_interval;
    return result;
}

void GreyOrangeSystem::set_total_sim_time(int _sim_time, int warmup_time) {
    throw std::runtime_error(
        "ManufactureSystem::set_total_sim_time Not implemented");
}

json GreyOrangeSystem::warmup(int warmup_time) {
    throw std::runtime_error("ManufactureSystem::warmup Not implemented");
}

json GreyOrangeSystem::update_gg_and_step(int update_gg_interval) {
    throw std::runtime_error(
        "ManufactureSystem::update_gg_and_step Not implemented");
}

bool GreyOrangeSystem::congested() const {
    if (simulation_window <= 1)
        return false;
    int wait_agents = 0;
    for (const auto &path : paths) {
        int t = 0;
        while (t < simulation_window &&
               path[timestep].location == path[timestep + t].location &&
               path[timestep].orientation == path[timestep + t].orientation &&
               !path[timestep + t].is_tasking_wait &&  // Rule out tasking wait
               !path[timestep + t].is_rotating)        // Rule out rotating
        {
            t++;
        }
        if (t == simulation_window)
            wait_agents++;
    }
    // more than half of drives didn't make progress
    return wait_agents > num_of_drives / 2;
}
json GreyOrangeSystem::simulate(int simulation_time) {
    std::cout << "*** Simulating " << seed << " ***" << std::endl;
    this->simulation_time = simulation_time;
    initialize();

    std::vector<std::vector<int>> tasks_finished_timestep;

    bool congested_sim = false;

    for (; timestep < simulation_time; timestep += simulation_window) {
        if (this->screen > 0)
            std::cout << "Timestep " << timestep << std::endl;

        update_start_locations(0);
        update_goal_locations(0);
        solve();

        // move drives
        auto new_finished_tasks = move();
        if (this->screen > 0)
            std::cout << new_finished_tasks.size() << " tasks has been finished"
                      << std::endl;

        // update tasks
        int n_tasks_finished_per_step = 0;
        for (auto task : new_finished_tasks) {
            int id, loc, t;
            id = task.agent_id;
            loc = task.location;
            // exclude wait time
            // t = task.finish_t - task.task_wait_time;
            t = task.finish_t;
            this->finished_tasks[id].emplace_back(task);
            this->num_of_tasks++;
            n_tasks_finished_per_step++;
            if (this->hold_endpoints)
                this->held_endpoints.erase(loc);
        }

        std::vector<int> curr_task_finished{n_tasks_finished_per_step,
                                            timestep};
        tasks_finished_timestep.emplace_back(curr_task_finished);

        // if (this->task_dist_update_interval > 0 && this->timestep %
        // this->task_dist_update_interval == 0){ this->update_task_dist();
        // }

        if (congested()) {
            cout << "***** Timestep " << timestep
                 << ": Too many traffic jams *****" << endl;
            congested_sim = true;
            if (this->stop_at_traffic_jam) {
                break;
            }
        }

        // Overtime?
        double runtime = (double)(clock() - this->start_time) / CLOCKS_PER_SEC;
        if (runtime >= this->overall_time_limit) {
            cout << "***** Timestep " << timestep << ": Overtime *****" << endl;
            break;
        }
    }

    json result = this->summarizeResult();
    result["tasks_finished_timestep"] = tasks_finished_timestep;
    result["congested"] = congested_sim;
    return result;
}

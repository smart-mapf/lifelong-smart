#include "SortingSystem.h"
#include "WHCAStar.h"
#include "ECBS.h"
#include "LRAStar.h"
#include "PBS.h"
#include "helper.h"
#include "common.h"
#include <algorithm>

SortingSystem::SortingSystem(SortingGrid &G,
                             MAPFSolver &solver,
                             // from destination to chutes
                             std::map<int, vector<int>> chute_mapping,
                             std::string package_mode,
                             // list of packages, essentially list of destinations
                             vector<int> packages,
                             // distribution of packages on each destination
                             vector<double> package_dist_weight,
                             // task assignment cost
                             std::string task_assignment_cost,
                             vector<double> task_assignment_params) :
                             BasicSystem(G, solver), G(G),
                             chute_mapping(chute_mapping),
                             package_mode(package_mode),
                             packages(packages),
                             package_dist_weight(package_dist_weight),
                             task_assignment_cost(task_assignment_cost),
                             task_assignment_params(task_assignment_params) {}

SortingSystem::~SortingSystem()
{
}


// next_pos: (next_loc, num_robots_targeting_next_loc)
double SortingSystem::compute_assignment_cost(
    int curr_loc, pair<int, int> next_pos) const
{
    double cost = -1;
    if (this->task_assignment_cost == "heuristic+num_agents")
    {
        cost = this->G.get_heuristic(next_pos.first, curr_loc) +
               this->assign_C * next_pos.second;
    }
    else if (this->task_assignment_cost == "heuristic")
    {
        cost = this->G.get_heuristic(next_pos.first, curr_loc);
    }
    else if (this->task_assignment_cost == "opt_quadratic_f")
    {
        // Cost is computed by a function
        // f(x, y) = a_0 * x^2 + a_1 y^2 + a_2 * xy + a_3* x + a_4 * y,
        // where x and y are heuristic and number of agents, respectively, and
        // a's are optimized parameters
        // Print content of G.heuristics
        double x = this->G.get_heuristic(next_pos.first, curr_loc);
        double y = next_pos.second;
        cost = this->task_assignment_params[0] * x * x +
               this->task_assignment_params[1] * y * y +
               this->task_assignment_params[2] * x * y +
               this->task_assignment_params[3] * x +
               this->task_assignment_params[4] * y;
    }
    else
    {
        std::cout << "unknown task assignment strategy" << std::endl;
        exit(-1);
    }
    return cost;
}


int SortingSystem::assign_workstation(int curr_loc) const
{
    // Choose a workstation based on:
    // heuristic[curr][next] + C * #robots targeting `next`
    int assigned_loc;
    double min_cost = DBL_MAX;

    // ONLYDEV(
    //     int x = curr_loc % map.cols;
    //     int y = curr_loc / map.cols;
    //     cout << "current loc: (" << x << ", " << y << ")" << endl;
    // )

    for (auto workstation : this->robots_in_workstations)
    {
        // cout << "workstation = " << workstation.first << endl;
        double cost = this->compute_assignment_cost(curr_loc, workstation);
        if (cost < min_cost)
        {
            min_cost = cost;
            assigned_loc = workstation.first;
        }
    }
    // ONLYDEV(
    //     x = assigned_loc % map.cols;
    //     y = assigned_loc / map.cols;
    //     cout << "assigned workstation: (" << x << ", " << y << ")" << endl;)
    return assigned_loc;
}

int SortingSystem::assign_endpoint(int curr_loc, vector<int> endpoints) const
{
    // Choose an endpoint based on:
    // heuristic[curr][next] + C * #robots targeting `next`
    int assigned_loc;
    double min_cost = DBL_MAX;

    for (auto endpoint : endpoints)
    {
        double cost = this->compute_assignment_cost(
            curr_loc,
            make_pair(endpoint, this->robots_in_endpoints.at(endpoint)));
        if (cost < min_cost)
        {
            min_cost = cost;
            assigned_loc = endpoint;
        }
    }
    return assigned_loc;
}


void SortingSystem::initialize()
{
    cout << "Initializing SortingSystem" << endl;
	initialize_solvers();

	starts.resize(num_of_drives);
	goal_locations.resize(num_of_drives);
	paths.resize(num_of_drives);
	finished_tasks.resize(num_of_drives);
    waited_time.resize(num_of_drives, 0);
    is_tasking.resize(num_of_drives, false);
    rotate_time.resize(num_of_drives, 0);
    is_rotating.resize(num_of_drives, false);

    // Initialize package distribution
    if (this->package_mode == "dist")
    {
        this->package_dist = std::discrete_distribution<int>(
            package_dist_weight.begin(),
            package_dist_weight.end());
        cout << "package distribution: ";
        for (auto w : package_dist_weight)
        {
            cout << w << ", ";
        }
        cout << endl;
    }

    // Initialize number of robots going to each workstation
    cout << "workstations size = " << this->G.workstations.size() << endl;
    for (auto workstation : this->G.workstations)
    {
        this->robots_in_workstations[workstation] = 0;
    }

    // Initialize number of robots going to each endpoint
    cout << "endpoints size = " << this->G.endpoints.size() << endl;
    for (auto endpoint : this->G.endpoints)
    {
        this->robots_in_endpoints[endpoint] = 0;
    }
    // Print content of robots_in_workstations and robots_in_endpoints
    cout << "robots_in_workstations: ";
    for (auto &r : robots_in_workstations)
    {
        cout << r.second << ", ";
    }
    cout << endl;
    cout << "robots_in_endpoints: ";
    for (auto &r : robots_in_endpoints)
    {
        cout << r.second << ", ";
    }
    cout << endl;

    // Shuffle the packages
    std::shuffle(this->packages.begin(), this->packages.end(), this->gen);


	bool succ = load_records(); // continue simulating from the records
	if (succ){
		std::cout << "load_records = "<<succ<<", timestep = "<<timestep<<std::endl;
	}
	// exit(-1);

	if (!succ)
	{
		timestep = 0;
		succ = load_locations();
		if (!succ)
		{
			cout << "Randomly generating initial start locations" << endl;
			initialize_start_locations();
            cout << "Randomly generating initial goal locations" << endl;
			initialize_goal_locations();
		}
	}

	// Initialize next_goal_type to "w" under w mode
	if (G.get_w_mode())
    {
        for (int i = 0; i < num_of_drives; i++)
        {
            int start_loc = this->starts[i].location;
            // If start from workstation, next goal is endpoint
            if (std::find(
                    G.workstations.begin(),
                    G.workstations.end(),
                    start_loc) != G.workstations.end())
            {
                this->next_goal_type.push_back("e");
            }
            // Otherwise, next goal is workstation
            else
            {
                this->next_goal_type.push_back("w");
            }
        }

        // Create workstation distribution
        // Number of weight must be the same as number of workstations
        // assert(this->G.workstation_weights.size() ==
        //        this->G.workstations.size());

        // std::random_device rd;
        this->gen = mt19937(this->seed);

        // this->workstation_dist = discrete_distribution<int>(
        //     this->G.workstation_weights.begin(),
        //     this->G.workstation_weights.end());
        // cout << "Workstation distribution: ";
        // for (auto w: this->G.workstation_weights)
        // {
        //     cout << w << ", ";
        // }
        // cout << endl;

		// // initialize end_points_weights
		// this->G.initialize_end_points_weights();
		// this->end_points_dist = discrete_distribution<int>(
		// 	this->G.end_points_weights.begin(),
		// 	this->G.end_points_weights.end()
		// );

    }
}

void SortingSystem::initialize_start_locations()
{
	// Choose random start locations
	// Any non-obstacle locations can be start locations
	// Start locations should be unique
	for (int k = 0; k < num_of_drives; k++)
	{
		int orientation = -1;
		if (consider_rotation)
		{
			orientation = rand() % 4;
		}
		starts[k] = State(G.agent_home_locations[k], 0, orientation);
		paths[k].emplace_back(starts[k]);
		finished_tasks[k].emplace_back(
            Task(G.agent_home_locations[k], -1, 0, 0, k, 0));
	}
}

void SortingSystem::update_start_locations(int t)
{
    // std::cout << "in update start location, timestep = "<<timestep<<std::endl;
    for (int k = 0; k < num_of_drives; k++)
    {
        starts[k] = State(paths[k][timestep+t].location, 0, paths[k][timestep+t].orientation);
        // std::cout << "agent "<<k<<": loc="<<starts[k].location<<std::endl;
    }
}


// Sample the next package and decide the list of endpoints based on chute
// mapping
vector<int> SortingSystem::sample_package()
{
    // next task would be an endpoint determined by the packages
    // and chute mapping
    int next_package;
    if (this->package_mode == "explicit")
    {
        if (this->package_id >= this->packages.size())
        {
            std::cout << "not enough packages" << std::endl;
            exit(-1);
        }
        next_package = this->packages[this->package_id];
    }
    else if (this->package_mode == "dist")
    {
        next_package = this->package_dist(this->gen);
    }
    else
    {
        std::cout << "unkonw package mode" << std::endl;
        exit(-1);
    }

    this->package_id++;

    vector<int> endpoints;
    for (auto chute : this->chute_mapping[next_package])
    {
        for (auto endpoint : this->G.obs_adj_endpoints[chute])
        {
            endpoints.push_back(endpoint);
        }
    }
    return endpoints;
}


void SortingSystem::initialize_goal_locations()
{
    cout << "Initializing goal locations" << endl;
	if (hold_endpoints || useDummyPaths)
		return;
    // Choose random goal locations
    // a close induct location can be a goal location, or
    // any eject locations can be goal locations
    // Goal locations are not necessarily unique
    for (int k = 0; k < num_of_drives; k++)
    {
		int goal;
        // If the agent starts from a workstation, the goal is an endpoint
        if (G.types[starts[k].location] == "Workstation")
        {
            // cout << "agent " << k << " starts from " << endl;
            vector<int> endpoints = sample_package();
            // cout << "endpoints size = " << endpoints.size() << endl;
            goal = assign_endpoint(starts[k].location, endpoints);
            this->robots_in_endpoints[goal]++;
        }
        else // Otherwise, go to a workstation
        {
            // cout << "agent " << k << " starts from " << starts[k].location << " of type " << G.types[starts[k].location] << endl;
            goal = assign_workstation(starts[k].location);
            this->robots_in_workstations[goal]++;
        }
		goal_locations[k].emplace_back(goal, 0, 0);
    }
}


// void SortingSystem::initialize_goal_locations()
// {
// 	cout << "Initializing goal locations" << endl;
// 	if (hold_endpoints || useDummyPaths)
// 		return;
// 	// Choose random goal locations
// 	// Goal locations are not necessarily unique
// 	for (int k = 0; k < num_of_drives; k++)
// 	{
// 		int goal = G.endpoints[rand() % (int)G.endpoints.size()];
// 		goal_locations[k].emplace_back(goal, 0, 0);
// 		// std::cout << "agent "<<k<<" init goal is "<< goal<<std::endl;
// 	}
// }

// void SortingSystem::update_task_dist(){
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

// int SortingSystem::sample_end_points(){
// 	int idx = this->end_points_dist(this->gen);
// 	return this->G.endpoints[idx];
// }

// int SortingSystem::sample_workstation()
// {
//     // Sample a workstation based the given weight
//     int idx = this->workstation_dist(this->gen);

//     return this->G.workstations[idx];
// }


int SortingSystem::gen_next_goal(int agent_id, int last_goal_loc)
{
	int next = -1;
    if (this->next_goal_type[agent_id] == "w")
    {
        next = assign_workstation(last_goal_loc);
        this->next_goal_type[agent_id] = "e";
        this->robots_in_workstations[last_goal_loc]++;
    }
    else if (this->next_goal_type[agent_id] == "e")
    {
        vector<int> endpoints = sample_package();
        next = assign_endpoint(last_goal_loc, endpoints);
        this->next_goal_type[agent_id] = "w";
        this->robots_in_endpoints[last_goal_loc]++;
    } else {
        std::cout << "error! next goal type is not w or e, but "<<this->next_goal_type[agent_id]<<std::endl;
        exit(1);
    }
	return next;
}

void SortingSystem::update_goal_locations(int t)
{
    cout << "Updating goal locations" << endl;
    if (!this->rule_based_called)
        new_agents.clear();

    // RHCR Algorithm
    for (int k = 0; k < num_of_drives; k++)
    {
        int curr = paths[k][timestep+t].location; // current location
        Task goal; // The last goal location
        if (goal_locations[k].empty())
        {
            // goal = make_tuple(curr, 0, 0);
            goal = Task(curr, -1, 0, 0);
        }
        else
        {
            goal = goal_locations[k].back();
        }
        int last_goal_loc = goal.location;
        double min_timesteps = G.get_Manhattan_distance(last_goal_loc, curr);
        while (min_timesteps <= simulation_window)
        // The agent might finish its tasks during the next planning horizon
        {
            // assign a new task
            Task next;
            if (G.types[last_goal_loc] == "Endpoint" ||
                G.types[last_goal_loc] == "Workstation")
            {
                // next = make_tuple(this->gen_next_goal(k, last_goal_loc), 0, 0);
                next = Task(this->gen_next_goal(k, last_goal_loc), -1, 0, 0);
            }
            else
            {
                std::cout << "ERROR in update_goal_function()" << std::endl;
                std::cout << "The fiducial type at curr="<<curr<<" should not be " << G.types[curr] << std::endl;
                exit(-1);
            }
            goal_locations[k].emplace_back(next);
            min_timesteps += G.get_Manhattan_distance(
                next.location, last_goal_loc);
            goal = next;
        }
    }
    cout << "End of updating goal locations" << endl;
}

tuple<vector<double>, double, double> SortingSystem::edge_pair_usage_mean_std(
    vector<vector<double>> &edge_usage)
{
    // For each pair of valid edges (i, j) and (j, i), calculate the absolute
    // of their edge usage difference, and calculate mean and std.
    vector<double> edge_pair_usage(this->G.get_n_valid_edges() / 2, 0.0);
    int valid_edge_id = 0;
    int n_vertices = this->G.rows * this->G.cols;
    for (int i = 0; i < n_vertices; i++)
	{
        // Start from i+1 to ignore wait actions
        for (int j = i + 1; j < n_vertices; j++)
        {
            if (this->G.types[i] != "Obstacle" &&
                this->G.types[j] != "Obstacle" &&
                this->G.get_Manhattan_distance(i, j) <= 1)
            {
                edge_pair_usage[valid_edge_id] = std::abs(
                    edge_usage[i][j] - edge_usage[j][i]);
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

tuple<vector<vector<vector<double>>>, vector<vector<double>>> SortingSystem::convert_edge_usage(vector<vector<double>> &edge_usage)
{
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
    for (int i = 0; i < n_vertices; i++)
	{
        // Start from i+1 to ignore wait actions
        for (int j = 0; j < n_vertices; j++)
        {
            int row_ = this->G.getRowCoordinate(i);
            int col_ = this->G.getColCoordinate(i);

            // Increment wait action usage matrix
            if (i == j)
            {
                vertex_wait_matrix[row_][col_] = edge_usage[i][j];
            }
            // Otherwise, if the edge is valid, update edge usage matrix
            else if (this->G.types[i] != "Obstacle" &&
                this->G.types[j] != "Obstacle" &&
                this->G.get_Manhattan_distance(i, j) <= 1)
            {
                // Obtain direction of edge (i, j)
                int dir = this->G.get_direction(i, j);

                // Set corresponding entry of edge usage matrix
                edge_usage_matrix[row_][col_][dir] = edge_usage[i][j];
            }
        }
	}
    return make_tuple(edge_usage_matrix, vertex_wait_matrix);
}

json SortingSystem::summarizeResult(){
	// Compute objective
	double throughput = (double)this->num_of_tasks / this->simulation_time;

	// Compute measures:
	// 1. Variance of tile usage
	// 2. Average number of waiting agents at each timestep
	// 3. Average distance of the finished tasks
    // 4. Average number of turns over the agents
    // 5. Average number of "reversed" action (not going in the direction
    //    suggested by highway) in each timestep, weighted by absolute value of
    //    the difference of edges both directions
    // 6. Variance of edge usage weighted by absolute value of the difference
    //    of edges both directions
    int n_vertices = this->G.rows * this->G.cols;
	std::vector<double> tile_usage(n_vertices, 0.0);
    // Adj matrix: [i,j] stores edge usage of edge from node_i to node_j
    std::vector<vector<double>> edge_usage(
        n_vertices, std::vector<double>(n_vertices, 0.0));
	std::vector<double> num_wait(this->simulation_time, 0.0);
	std::vector<double> finished_task_len;
    std::vector<double> num_turns(num_of_drives, 0.0);
    std::vector<double> num_rev_action(this->simulation_time, 0.0);
	for (int k = 0; k < num_of_drives; k++)
	{
		int path_length = this->paths[k].size();
		for (int j = 0; j < path_length; j++)
		{
			State s = this->paths[k][j];

			// Count tile usage
			tile_usage[s.location] += 1.0;

			if (j < path_length - 1)
			{
				State next_s = this->paths[k][j + 1];
                // See if action is stay
				if (s.location == next_s.location)
				{
                    // The planned path might go beyond the simulation window.
					if (s.timestep < this->simulation_time)
					{
						num_wait[s.timestep] += 1.0;
					}
				}

                // Edge weight from s_t to s_t+1
                double t_to_t_p_1 = this->G.get_weight(
                    s.location, next_s.location);
                // Edge weight from s_t+1 to s_t
                double t_p_1_to_t = this->G.get_weight(
                    next_s.location, s.location);

                // Increment edge usage.
                // This DOES include number of wait action at each vertex (the
                // diagnal of the matrix), but they will be ignored while
                // calculating the mean and std.
                edge_usage[s.location][next_s.location] += 1.0;

                // See if the action follows the edge direction "suggested" by
                // the highway system.
                // If the edge weight that the agent is traversing through is
                // larger than the direction in reverse, then the agent is NOT
                // following the suggested direction.
                if (s.timestep < this->simulation_time &&
                    s.location != next_s.location && t_to_t_p_1 > t_p_1_to_t)
                {
                    num_rev_action[s.timestep] += std::abs(t_to_t_p_1 - t_p_1_to_t);
                }
			}

            // Count the number of turns
            // See if the previous state and the next state change different
            // axis
            if (j > 0 && j < path_length - 1)
            {
                State prev_state = this->paths[k][j-1];
                State next_state = this->paths[k][j+1];
                int prev_row = this->G.getRowCoordinate(prev_state.location);
                int next_row = this->G.getRowCoordinate(next_state.location);
                int prev_col = this->G.getColCoordinate(prev_state.location);
                int next_col = this->G.getColCoordinate(next_state.location);
                if (prev_row != next_row && prev_col != next_col)
                {
                    num_turns[k] += 1;
                }
            }
		}

		int prev_t = 0;
		for (auto task : this->finished_tasks[k])
		{
			if (task.finish_t != 0)
			{
				int curr_t = task.finish_t;
				Path p = this->paths[k];

				// Calculate length of the path associated with this task
				double task_path_len = 0.0;
				for (int t = prev_t; t < curr_t - 1; t++)
				{
					if (p[t].location != p[t + 1].location)
					{
						task_path_len += 1.0;
					}
				}
				finished_task_len.push_back(task_path_len);
				prev_t = curr_t;
			}
		}
	}

    // Longest common sub-path
    vector<int> subpath = helper::longest_common_subpath(
        this->paths, this->simulation_time);

	// Post process data
	// Normalize tile usage s.t. they sum to 1
	double tile_usage_sum = helper::sum(tile_usage);
	helper::divide(tile_usage, tile_usage_sum);

	double tile_usage_mean, tile_usage_std;
	double edge_pair_usage_mean, edge_pair_usage_std;
    vector<double> edge_pair_usage;
    double num_wait_mean, num_wait_std;
	double finished_len_mean, finished_len_std;
    double num_turns_mean, num_turns_std;
    double num_rev_action_mean, num_rev_action_std;
    // double avg_task_len = this->G.get_avg_task_len(this->G.heuristics);
    vector<vector<vector<double>>> edge_usage_matrix;
    vector<vector<double>> vertex_wait_matrix;

	std::tie(tile_usage_mean, tile_usage_std) = helper::mean_std(tile_usage);
    std::tie(edge_pair_usage, edge_pair_usage_mean, edge_pair_usage_std) = edge_pair_usage_mean_std(edge_usage);
	std::tie(num_wait_mean, num_wait_std) = helper::mean_std(num_wait);
	std::tie(finished_len_mean, finished_len_std) = helper::mean_std(finished_task_len);
    std::tie(num_turns_mean, num_turns_std) = helper::mean_std(num_turns);
    std::tie(num_rev_action_mean, num_rev_action_std) = helper::mean_std(num_rev_action);
    std::tie(edge_usage_matrix, vertex_wait_matrix) = convert_edge_usage(edge_usage);

	// Log some of the results
	std::cout << std::endl;
	std::cout << "Throughput: " << throughput << std::endl;
	std::cout << "Std of tile usage: " << tile_usage_std << std::endl;
    std::cout << "Std of edge pair usage: " << edge_pair_usage_std << std::endl;
	std::cout << "Average wait at each timestep: " << num_wait_mean << std::endl;
	std::cout << "Average path length of each finished task: " << finished_len_mean << std::endl;
    // std::cout << "Average path length of each task: " << avg_task_len << std::endl;
    std::cout << "Average number of turns: " << num_turns_mean << std::endl;
    std::cout << "Average number of reversed actions in highway: " << num_rev_action_mean << std::endl;
    std::cout << "Length of longest common path: " << subpath.size()
              << std::endl;

	update_start_locations(0);
	std::cout << std::endl
			  << "Done!" << std::endl;
	save_results();

	// Create the result json object
	json result;
	result = {
		{"throughput", throughput},
		{"tile_usage", tile_usage},
        {"edge_pair_usage", edge_pair_usage},
        {"edge_usage_matrix", edge_usage_matrix},
        {"vertex_wait_matrix", vertex_wait_matrix},
		{"num_wait", num_wait},
        {"num_turns", num_turns},
        {"num_rev_action", num_rev_action},
		{"finished_task_len", finished_task_len},
		{"tile_usage_mean", tile_usage_mean},
		{"tile_usage_std", tile_usage_std},
        {"edge_pair_usage_mean", edge_pair_usage_mean},
        {"edge_pair_usage_std", edge_pair_usage_std},
		{"num_wait_mean", num_wait_mean},
		{"num_wait_std", num_wait_std},
		{"finished_len_mean", finished_len_mean},
		{"finished_len_std", finished_len_std},
        {"num_turns_mean", num_turns_mean},
        {"num_turns_std", num_turns_std},
        {"num_rev_action_mean", num_rev_action_mean},
        {"num_rev_action_std", num_rev_action_std},
        // {"tasks_finished_timestep", tasks_finished_timestep},
        // {"avg_task_len", avg_task_len},
        // {"congested", congested_sim},
        {"longest_common_path", subpath},
	};
	return result;
}

json SortingSystem::summarizeCurrResult(int summarize_interval){
	json all_paths = json::array();
	json full_paths = json::array();
	json future_paths = json::array();
	for (int k = 0; k < num_of_drives; k++){
		int path_length = this->paths[k].size();
		if (summarize_interval > timestep || summarize_interval + 1 > path_length){
			std::cout << "warning: summarize interval ["<<summarize_interval<<"],path length ["<<path_length<<"]!"<<std::endl;
			std::cout << "warning: timestep ["<<timestep<<"]!"<<std::endl;
			summarize_interval = min(path_length-1, timestep);
		}
		json agent_path = json::array();
		for (int j = timestep-summarize_interval; j <= timestep; j++){
			State s = this->paths[k][j];
			agent_path.push_back(s.location);
		}
		all_paths.push_back(agent_path);

		json full_path = json::array();
		for (int j = 0; j < path_length; j++){
			State s = this->paths[k][j];
			full_path.push_back(s.location);
		}
		full_paths.push_back(full_path);

		json future_path = json::array();
		for (int j = timestep; j < path_length; j++){
			State s = this->paths[k][j];
			future_path.push_back(s.location);
		}
		future_paths.push_back(future_path);
	}

	json goal_locs = json::array();
	for (int i=0; i<this->goal_locations.size(); ++i){
		int goal_loc = this->goal_locations[i][0].location;
		goal_locs.push_back(goal_loc);
	}

	json full_goal_locs = json::array();
	for (int i=0; i<this->goal_locations.size(); ++i){
		json full_agent_locs = json::array();
		for (int j=0; j<this->goal_locations[i].size(); ++j){
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

void SortingSystem::set_total_sim_time(int _sim_time, int warmup_time){
	this->total_sim_time = _sim_time + warmup_time;
}

json SortingSystem::warmup(int warmup_time){
	std::cout << "*** Simulating " << seed << " ***" << std::endl;
	// indicate current timestep
	this->simulation_time = warmup_time; // TODO: 看一下有什么用，可能和 log 相关
	initialize();

	std::vector<std::vector<int>> tasks_finished_timestep;

    bool congested_sim = false;
	bool timeout = false;

	int start_timestep = timestep;

	update_start_locations(0);
	if ((timestep<this->simulation_time) && (timestep<this->total_sim_time)){
		update_goal_locations(0);
	}
	// std::cout << "simulation_win = "<< simulation_window<<std::endl;
	for (; (timestep<this->simulation_time) && (timestep<this->total_sim_time); timestep += simulation_window)
	{
		if (this->screen > 0)
			std::cout << "Warm up: Timestep " << timestep << std::endl;

		solve();

		// move drives
		auto new_finished_tasks = move();
		if (this->screen > 0)
			std::cout << new_finished_tasks.size() << " tasks has been finished" << std::endl;

		// update tasks
        int n_tasks_finished_per_step = 0;
		for (auto task : new_finished_tasks)
		{
			int id, loc, t;
            id = task.agent_id;
            loc = task.location;
            t = task.finish_t;
			this->finished_tasks[id].emplace_back(task);
			// this->num_of_tasks++;
            n_tasks_finished_per_step++;
            this->update_n_agents(task);
		}

        std::vector<int> curr_task_finished {
            n_tasks_finished_per_step, timestep};
        tasks_finished_timestep.emplace_back(curr_task_finished);

		// if (this->task_dist_update_interval > 0 && this->timestep % this->task_dist_update_interval == 0){
		// 	this->update_task_dist();
		// }
		update_start_locations(simulation_window);
		update_goal_locations(simulation_window);
        this->check_n_agents_sum();

		if (congested())
		{
			cout << "***** Timestep " << timestep
                 << ": Too many traffic jams *****" << endl;
            congested_sim = true;
            if (this->stop_at_traffic_jam)
            {
                break;
            }
		}

        // Overtime?
        double runtime = (double)(clock() - this->start_time)/ CLOCKS_PER_SEC;
        if (runtime >= this->overall_time_limit)
        {
			timeout = true;
            cout << "***** Timestep " << timestep << ": Overtime *****" << endl;
            break;
        }
	}
	json result = this->summarizeCurrResult(timestep - start_timestep);
	result["congested"] = congested_sim;
	result["timeout"] = timeout;
	return result;
}

json SortingSystem::update_gg_and_step(int update_gg_interval){
	std::cout << "*** Simulating " << seed << " ***" << std::endl;
	// this->simulation_time = timestep + update_gg_interval;
	this->simulation_time += update_gg_interval;
	// std::cout << "simulation time = "<<this->simulation_time << std::endl;


	std::vector<std::vector<int>> tasks_finished_timestep;

    bool congested_sim = false;
	bool timeout = false;

	int start_timestep = timestep;
	if (this->screen > 0)
		std::cout << "Timestep " << timestep << std::endl;

	for (; (timestep<this->simulation_time) && (timestep<this->total_sim_time); timestep += simulation_window)
	{
		if (this->screen > 0)
			std::cout << "Update: Timestep " << timestep << std::endl;

		// std::fstream output;
		// output.open(outfile + "/detail_before" + std::to_string(timestep+simulation_window) + ".txt", std::ios::out);
		// for(int k=0; k<this->num_of_drives; ++k){
		// 	output << "agent "<<k<<std::endl;
		// 	output << "start: "<<starts[k].location<<std::endl;
		// 	output << "goal: ";
		// 	for (int j=0; j<goal_locations[k].size(); ++j){
		// 		output << std::get<0>(goal_locations[k][j])<<", ";
		// 	}
		// 	output << std::endl;
		// }
		// output.close();

		// output.open(outfile + "/weights_before" + std::to_string(timestep+simulation_window) + ".txt", std::ios::out);
		// output << "***weights***" << std::endl;
        // for (std::vector<double> n : this->G.weights)
        // {
        //     for (double w : n)
        //     {
        //         output << w << ",";
        //     }
        //     output << std::endl;
        // }

		if (this->screen > 0){
			std::cout << "solve" <<std::endl;
		}
		solve();


		if (this->screen > 0){
			std::cout << "move" << std::endl;
		}

		// move drives
		auto new_finished_tasks = move();
		if (this->screen > 0)
			std::cout << new_finished_tasks.size() << " tasks has been finished" << std::endl;

		// update tasks
        int n_tasks_finished_per_step = 0;
		for (auto task : new_finished_tasks)
		{
			int id, loc, t;
            id = task.agent_id;
            loc = task.location;
            t = task.finish_t;
			this->finished_tasks[id].emplace_back(task);
			this->num_of_tasks++;
            n_tasks_finished_per_step++;
            this->update_n_agents(task);
		}

        std::vector<int> curr_task_finished {
            n_tasks_finished_per_step, timestep};
        tasks_finished_timestep.emplace_back(curr_task_finished);

		// if (this->task_dist_update_interval > 0 && this->timestep % this->task_dist_update_interval == 0){
		// 	this->update_task_dist();
		// }

		// exit(1);

		update_start_locations(simulation_window);
		if (this->screen > 0){
			std::cout << "update goal locations" <<std::endl;
		}
		update_goal_locations(simulation_window);
        this->check_n_agents_sum();

		// output.open(outfile + "/detail" + std::to_string(timestep+simulation_window) + ".txt", std::ios::out);
		// for(int k=0; k<this->num_of_drives; ++k){
		// 	output << "agent "<<k<<std::endl;
		// 	output << "start: "<<starts[k].location<<std::endl;
		// 	output << "goal: ";
		// 	for (int j=0; j<goal_locations[k].size(); ++j){
		// 		output << std::get<0>(goal_locations[k][j])<<", ";
		// 	}
		// 	output << std::endl;
		// }
		// output.close();

		this->save_results();

		if (congested())
		{
			cout << "***** Timestep " << timestep
                 << ": Too many traffic jams *****" << endl;
            congested_sim = true;
            if (this->stop_at_traffic_jam)
            {
                break;
            }
		}

        // Overtime?
        double runtime = (double)(clock() - this->start_time)/ CLOCKS_PER_SEC;
        if (runtime >= this->overall_time_limit)
        {
			timeout = true;
            cout << "***** Timestep " << timestep << ": Overtime *****" << endl;
            break;
        }
	}
	json result = this->summarizeCurrResult(timestep - start_timestep);
	result["congested"] = congested_sim;
	result["timeout"] = timeout;
	return result;
}

json SortingSystem::simulate(int simulation_time)
{
	std::cout << "*** Simulating " << seed << " ***" << std::endl;
	this->simulation_time = simulation_time;
	initialize();

    std::vector<std::vector<int>> tasks_finished_timestep;

    bool congested_sim = false;

	for (; timestep < simulation_time; timestep += simulation_window)
	{
		if (this->screen > 0)
			std::cout << "Timestep " << timestep << std::endl;

		update_start_locations(0);
		update_goal_locations(0);
		solve();

		// move drives
		auto new_finished_tasks = move();
		if (this->screen > 0)
			std::cout << new_finished_tasks.size() << " tasks has been finished" << std::endl;

		// update tasks
        int n_tasks_finished_per_step = 0;
		for (auto task : new_finished_tasks)
		{
			int id, loc, t;
            id = task.agent_id;
            loc = task.location;
            t = task.finish_t;
			this->finished_tasks[id].emplace_back(task);
			this->num_of_tasks++;
            n_tasks_finished_per_step++;
            this->update_n_agents(task);
		}

        std::vector<int> curr_task_finished {
            n_tasks_finished_per_step, timestep};
        tasks_finished_timestep.emplace_back(curr_task_finished);

		// if (this->task_dist_update_interval > 0 && this->timestep % this->task_dist_update_interval == 0){
		// 	this->update_task_dist();
		// }

		if (congested())
		{
			cout << "***** Timestep " << timestep
                 << ": Too many traffic jams *****" << endl;
            congested_sim = true;
            if (this->stop_at_traffic_jam)
            {
                break;
            }
		}

        // Overtime?
        double runtime = (double)(clock() - this->start_time)/ CLOCKS_PER_SEC;
        if (runtime >= this->overall_time_limit)
        {
            cout << "***** Timestep " << timestep << ": Overtime *****" << endl;
            break;
        }
	}

	json result = this->summarizeResult();
	result["tasks_finished_timestep"] = tasks_finished_timestep;
	result["congested"] = congested_sim;
	return result;
}


void SortingSystem::check_n_agents_sum()
{
    int sum = 0;
    for (auto & r: robots_in_workstations){
        sum += r.second;
    }
    for (auto & r: robots_in_endpoints){
        sum += r.second;
    }
    cout << "Sum of agents in workstations and endpoints: " << sum << endl;
}


void SortingSystem::update_n_agents(Task task)
{
    if (this->G.types[task.location] == "Workstation")
    {
        this->robots_in_workstations[task.location]--;
    }
    else if (this->G.types[task.location] == "Endpoint")
    {
        this->robots_in_endpoints[task.location]--;
    }
    else
    {
        cout << "task is at loc " << task.location << endl;
        cout << "task is of type " << this->G.types[task.location] << endl;
        cout << "ERROR: The task location is not a workstation or endpoint" << endl;
        exit(1);
    }
}

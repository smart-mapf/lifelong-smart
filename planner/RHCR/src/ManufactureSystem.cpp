#include "ManufactureSystem.h"
#include "WHCAStar.h"
#include "ECBS.h"
#include "LRAStar.h"
#include "PBS.h"
#include "helper.h"
#include "common.h"
#include <algorithm>

ManufactureSystem::ManufactureSystem(const ManufactureGrid &G, MAPFSolver &solver) : BasicSystem(G, solver), G(G) {}

ManufactureSystem::~ManufactureSystem()
{
}

void ManufactureSystem::initialize()
{
	initialize_solvers();

	starts.resize(num_of_drives);
	goal_locations.resize(num_of_drives);
	paths.resize(num_of_drives);
	finished_tasks.resize(num_of_drives);
    waited_time.resize(num_of_drives, 0);
    is_tasking.resize(num_of_drives, false);
    rotate_time.resize(num_of_drives, 0);
    is_rotating.resize(num_of_drives, false);
	bool succ = load_records(); // continue simulating from the records
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
}

void ManufactureSystem::initialize_start_locations()
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
		this->starts[k] = State(G.agent_home_locations[k], 0, orientation);
		this->paths[k].emplace_back(this->starts[k]);
		finished_tasks[k].emplace_back(
            Task(G.agent_home_locations[k], -1, 0, 0, k, 0));
	}
}

void ManufactureSystem::initialize_goal_locations()
{
	cout << "Initializing goal locations" << endl;

    // First goal must be the 0-th type
    int first_goal_type = 0;

	// Choose random goal locations
	// Goal locations are not necessarily unique
	for (int k = 0; k < num_of_drives; k++)
	{
        this->curr_goal_types.push_back(first_goal_type);
		int goal = G.endpoints[first_goal_type][
            rand() % (int)G.endpoints[first_goal_type].size()];
        int next_goal_wait = G.station_wait_time[first_goal_type];
		this->goal_locations[k].emplace_back(goal, 0, next_goal_wait);

        if (this->screen > 0)
        {
            cout << "Agent " << k <<  ": Next goal " << goal
                 << " with type " << first_goal_type
                 << ", should wait " << next_goal_wait << " timesteps"
                 << endl;
        }
    }
}


int ManufactureSystem::gen_next_goal(int agent_id, bool repeat_last_goal)
{
    int curr_goal_type = this->curr_goal_types[agent_id];
    int next_goal_type;
    if (repeat_last_goal)
    {
        next_goal_type = curr_goal_type;
    }
    else
    {
        next_goal_type = (curr_goal_type + 1) % this->G.n_station_types;
    }
    int next_goal = G.endpoints[next_goal_type][
        rand() % (int)G.endpoints[next_goal_type].size()];
    this->curr_goal_types[agent_id] = next_goal_type;
    return next_goal;
}


void ManufactureSystem::update_start_locations()
{
    // cout << "In update start locations " << endl;
    for (int k = 0; k < num_of_drives; k++)
    {
        // 1. Set start timestep for each agent to the number of timesteps it
        // needs to wait
        // 2. Insert proper number of wait actions at the start of the path
        // (updatePath in SIPP)

        // The agent is still doing task in the previous goal. Let it finish it.
        if (this->is_tasking[k])
        {
            int start_timestep = goal_locations[k].front().task_wait_time -
                                 this->waited_time[k];
            if (screen > 0)
            {
                cout << "Agent " << k << ": start_timestep = " << goal_locations[k].front().task_wait_time << " - " << this->waited_time[k] << ", start location = " << paths[k][timestep].location << endl;
            }
            assert(start_timestep > 0);

            starts[k] = State(
                paths[k][timestep].location,
                start_timestep,
                paths[k][timestep].orientation,
                true);
        }
        else
        {
            starts[k] = State(
                paths[k][timestep].location,
                0,
                paths[k][timestep].orientation);
        }

    }
}

void ManufactureSystem::update_goal_locations()
{
    if (!this->rule_based_called)
        new_agents.clear();
	// if (hold_endpoints)
	// {
	// 	unordered_map<int, int> held_locations; // <location, agent id>
	// 	for (int k = 0; k < num_of_drives; k++)
	// 	{
	// 		int curr = paths[k][timestep].location; // current location
	// 		if (goal_locations[k].empty())
	// 		{
	// 			int next = this->gen_next_goal(k);
	// 			while (next == curr || held_endpoints.find(next) != held_endpoints.end())
	// 			{
	// 				next = this->gen_next_goal(k, true);
	// 			}
	// 			goal_locations[k].emplace_back(next, 0);
	// 			held_endpoints.insert(next);
	// 		}
	// 		if (paths[k].back().location == goal_locations[k].back().first && // agent already has paths to its goal location
	// 			paths[k].back().timestep >= goal_locations[k].back().second)  // after its release time
	// 		{
	// 			int agent = k;
	// 			int loc = goal_locations[k].back().first;
	// 			auto it = held_locations.find(loc);
	// 			while (it != held_locations.end()) // its start location has been held by another agent
	// 			{
	// 				int removed_agent = it->second;
	// 				if (goal_locations[removed_agent].back().first != loc)
	// 					cout << "BUG" << endl;
	// 				new_agents.remove(removed_agent); // another agent cannot move to its new goal location
	// 				cout << "Agent " << removed_agent << " has to wait for agent " << agent << " because of location " << loc << endl;
	// 				held_locations[loc] = agent; // this agent has to keep holding this location
	// 				agent = removed_agent;
	// 				loc = paths[agent][timestep].location; // another agent's start location
	// 				it = held_locations.find(loc);
	// 			}
	// 			held_locations[loc] = agent;
	// 		}
	// 		else // agent does not have paths to its goal location yet
	// 		{
	// 			if (held_locations.find(goal_locations[k].back().first) == held_locations.end()) // if the goal location has not been held by other agents
	// 			{
	// 				held_locations[goal_locations[k].back().first] = k; // hold this goal location
	// 				new_agents.emplace_back(k);							// replan paths for this agent later
	// 				continue;
	// 			}
	// 			// the goal location has already been held by other agents
	// 			// so this agent has to keep holding its start location instead
	// 			int agent = k;
	// 			int loc = curr;
	// 			cout << "Agent " << agent << " has to wait for agent " << held_locations[goal_locations[k].back().first] << " because of location " << goal_locations[k].back().first << endl;
	// 			auto it = held_locations.find(loc);
	// 			while (it != held_locations.end()) // its start location has been held by another agent
	// 			{
	// 				int removed_agent = it->second;
	// 				if (goal_locations[removed_agent].back().first != loc)
	// 					cout << "BUG" << endl;
	// 				new_agents.remove(removed_agent); // another agent cannot move to its new goal location
	// 				cout << "Agent " << removed_agent << " has to wait for agent " << agent << " because of location " << loc << endl;
	// 				held_locations[loc] = agent; // this agent has to keep holding its start location
	// 				agent = removed_agent;
	// 				loc = paths[agent][timestep].location; // another agent's start location
	// 				it = held_locations.find(loc);
	// 			}
	// 			held_locations[loc] = agent; // this agent has to keep holding its start location
	// 		}
	// 	}
	// }

    // RHCR Algorithm
    for (int k = 0; k < num_of_drives; k++)
    {
        int curr = paths[k][timestep].location; // current location
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

        // Incorporate task wait time in expected `min_timesteps`
        double min_timesteps = G.get_Manhattan_distance(
            goal.location, curr) + goal.task_wait_time;

        // We should add goal until there are at least 2 goals for the agent.
        // This is because if the agent has arrived at the only goal of it and
        // has started doing task on it, the MAPF planner will ignore that
        // goal. The MAPF planner will not plan any thing which triggers error.
        // So a hacky way to solve this problem is adding another goal, but it
        // is not very efficient because we are essentially planning things
        // that are not necessary.
        while (min_timesteps <= simulation_window ||
               goal_locations[k].size() < 2)
        // The agent might finish its tasks during the next planning horizon
        {
            // assign a new task
            Task next;
            if (G.types[goal.location] == "Endpoint")
            {
                // In manufacturing scenario, it is possible for two
                // consecutive goals to be at the same place.
                int next_goal_loc = this->gen_next_goal(k);
                int next_goal_wait =
                    G.station_wait_time[this->curr_goal_types[k]];
                // next = make_tuple(next_goal_loc, 0, next_goal_wait);
                next = Task(next_goal_loc, -1, next_goal_wait, 0);
                if (this->screen > 0)
                {
                    cout << "Agent " << k <<  ": Next goal " << next.location
                         << " with type " << this->curr_goal_types[k]
                         << ", should wait " << next_goal_wait << " timesteps"
                         << endl;
                }
            }
            else
            {
                std::cout << "ERROR in update_goal_function()" << std::endl;
                std::cout << "The fiducial type should not be " << G.types[curr] << std::endl;
                exit(-1);
            }
            goal_locations[k].emplace_back(next);
            // Incorporate task wait time in expected `min_timesteps`
            min_timesteps += G.get_Manhattan_distance(
                next.location, goal.location) + goal.task_wait_time;
            goal = next;
        }
    }
}

bool ManufactureSystem::congested() const
{
    if (simulation_window <= 1)
        return false;
    int wait_agents = 0;
    for (const auto &path : paths)
    {
        int t = 0;
        while (t < simulation_window &&
               path[timestep].location == path[timestep + t].location &&
               path[timestep].orientation == path[timestep + t].orientation &&
               !path[timestep + t].is_tasking_wait)// Rule out tasking wait
        {
            t++;
        }
        if (t == simulation_window)
            wait_agents++;
    }
    // more than half of drives didn't make progress
    return wait_agents > num_of_drives / 2;
}


json ManufactureSystem::simulate(int simulation_time)
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

		update_start_locations();
		update_goal_locations();
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
			// if (this->hold_endpoints)
			// 	this->held_endpoints.erase(loc);
		}

        std::vector<int> curr_task_finished {
            n_tasks_finished_per_step, timestep};
        tasks_finished_timestep.emplace_back(curr_task_finished);

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

	// Compute objective
	double throughput = (double)this->num_of_tasks / this->simulation_time;

	// Compute measures:
	// 1. Variance of tile usage
	// 2. Average number of waiting agents at each timestep
	// 3. Average distance of the finished tasks
    // 4. Average number of turns over the agents
	std::vector<double> tile_usage(this->G.rows * this->G.cols, 0.0);
	std::vector<double> num_wait(this->simulation_time, 0.0);
	std::vector<double> finished_task_len;
    std::vector<double> num_turns(num_of_drives, 0.0);
	for (int k = 0; k < num_of_drives; k++)
	{
		int path_length = this->paths[k].size();
		for (int j = 0; j < path_length; j++)
		{
			State s = this->paths[k][j];

			// Count tile usage
			tile_usage[s.location] += 1.0;

			// See if action is stay
			if (j < path_length - 1)
			{
				State next_s = this->paths[k][j + 1];
				if (s.location == next_s.location)
				{
					if (s.timestep < this->simulation_time)
					{
						num_wait[s.timestep] += 1.0;
					}
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

	// Post process data
	// Normalize tile usage s.t. they sum to 1
	double tile_usage_sum = helper::sum(tile_usage);
	helper::divide(tile_usage, tile_usage_sum);

	double tile_usage_mean, tile_usage_std;
	double num_wait_mean, num_wait_std;
	double finished_len_mean, finished_len_std;
    double num_turns_mean, num_turns_std;
    // double avg_task_len = this->G.get_avg_task_len(this->G.heuristics);

	std::tie(tile_usage_mean, tile_usage_std) = helper::mean_std(tile_usage);
	std::tie(num_wait_mean, num_wait_std) = helper::mean_std(num_wait);
	std::tie(finished_len_mean, finished_len_std) = helper::mean_std(finished_task_len);
    std::tie(num_turns_mean, num_turns_std) = helper::mean_std(num_turns);

	// Log some of the results
	std::cout << std::endl;
	std::cout << "Throughput: " << throughput << std::endl;
	std::cout << "Std of tile usage: " << tile_usage_std << std::endl;
	std::cout << "Average wait at each timestep: " << num_wait_mean << std::endl;
	std::cout << "Average path length of each finished task: " << finished_len_mean << std::endl;
    // std::cout << "Average path length of each task: " << avg_task_len << std::endl;
    std::cout << "Average number of turns: " << num_turns_mean << std::endl;

	update_start_locations();
	std::cout << std::endl
			  << "Done!" << std::endl;
	save_results();

	// Create the result json object
	json result;
	result = {
		{"throughput", throughput},
		{"tile_usage", tile_usage},
		{"num_wait", num_wait},
        {"num_turns", num_turns},
		{"finished_task_len", finished_task_len},
		{"tile_usage_mean", tile_usage_mean},
		{"tile_usage_std", tile_usage_std},
		{"num_wait_mean", num_wait_mean},
		{"num_wait_std", num_wait_std},
		{"finished_len_mean", finished_len_mean},
		{"finished_len_std", finished_len_std},
        {"num_turns_mean", num_turns_mean},
        {"num_turns_std", num_turns_std},
        {"tasks_finished_timestep", tasks_finished_timestep},
        // {"avg_task_len", avg_task_len},
        {"congested", congested_sim},
        {"n_mapf_calls", this->n_mapf_calls},
        {"n_rule_based_calls", this->n_rule_based_calls}
	};
	return result;
}

json ManufactureSystem::warmup(int warmup_time)
{
    throw std::runtime_error("ManufactureSystem::warmup Not implemented");
}

json ManufactureSystem::update_gg_and_step(int update_gg_interval)
{
    throw std::runtime_error("ManufactureSystem::update_gg_and_step Not implemented");
}

void ManufactureSystem::set_total_sim_time(int total_sim_time, int warmup_time)
{
    throw std::runtime_error("ManufactureSystem::set_total_sim_time Not implemented");
}
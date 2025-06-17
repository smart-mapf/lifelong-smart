#include "BasicSystem.h"
#include <stdlib.h>
#include <boost/tokenizer.hpp>

BasicSystem::BasicSystem(const BasicGraph &G, MAPFSolver &solver) : G(G), solver(solver), num_of_tasks(0) {}

BasicSystem::~BasicSystem() {}

// TODO: implement the random instance generator
/*bool BasicSystem::load_config(std::string fname)
{
    std::string line;
    std::ifstream myfile(fname.c_str());
    if (!myfile.is_open()) {
        std::cout << "Config file " << fname << " does not exist. " << std::endl;
        return false;
    }
    getline(myfile, line);
    boost::char_separator<char> sep(" ");
    boost::tokenizer<boost::char_separator<char> > tok(line, sep);
    boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin();
    duration = atoi((*beg).c_str());
    getline(myfile, line);
    tok.assign(line, sep);
    fiducial = atoi((*beg).c_str());
    getline(myfile, line);
    tok.assign(line, sep);
    double length = atoi((*beg).c_str()) / fiducial;
    getline(myfile, line);
    tok.assign(line, sep);
    double width = atoi((*beg).c_str()) / fiducial;
    getline(myfile, line);
    tok.assign(line, sep);
    double v_max = atoi((*beg).c_str()) * duration / fiducial / 1000;
    getline(myfile, line);
    tok.assign(line, sep);
    double a_max = atoi((*beg).c_str()) * duration * duration / fiducial / 1000000;
    getline(myfile, line);
    tok.assign(line, sep);
    double rotate90 = atoi((*beg).c_str()) / duration;
    getline(myfile, line);
    tok.assign(line, sep);
    int rotate180 = atoi((*beg).c_str()) / duration;
    getline(myfile, line);
    tok.assign(line, sep);
    planning_window = atoi((*beg).c_str()) / duration;
    getline(myfile, line);
    tok.assign(line, sep);
    simulation_time = atoi((*beg).c_str()) / duration;

    myfile.close();
return true;

}
bool BasicSystem::generate_random_MAPF_instance()
{
    std::cout << "*** Generating instance " << seed << " ***" << std::endl;
    clock_t t = std::clock();
    // initialize_start_locations();
    // initialize_goal_locations();
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
    // print_MAPF_instance();

    return true;
}


void BasicSystem::print_MAPF_instance() const
{
    std::cout << "*** instance " << seed << " ***" << std::endl;
    for (int k = 0; k < (int)starts.size(); k++)
    {
        cout << "Agent " << k << ": " << starts[k];
        for (int goal : goal_locations[k])
            cout << "->" << goal;
        cout << endl;
    }
}


void BasicSystem::save_MAPF_instance(std::string fname) const
{
    std::ofstream stats;
    stats.open(fname, std::ios::app);
    stats << starts.size();
    for (int k = 0; k < (int)starts.size(); k++)
    {
        stats << k << "," << starts[k].location;
        for (int goal : goal_locations[k])
            cout << "," << goal;
        cout << endl;
    }
    stats.close();
}

bool BasicSystem::read_MAPF_instance(std::string fname)
{
    std::string line;
    std::ifstream myfile(fname.c_str());
    if (!myfile.is_open()) {
        std::cout << "MAPF instance file " << fname << " does not exist. " << std::endl;
        return false;
    }

    std::cout << "*** Reading instance " << fname << " ***" << std::endl;

    getline(myfile, line);
    boost::char_separator<char> sep(",");
    boost::tokenizer<boost::char_separator<char> > tok(line, sep);
    boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin();
    this->num_of_drives = atoi((*beg).c_str()); // read number of cols
    this->starts.resize(num_of_drives);
    this->goal_locations.resize(num_of_drives);

    for (int i = 0; i < num_of_drives; i++) {
        getline(myfile, line);
        boost::tokenizer<boost::char_separator<char> > tok(line, sep);
        beg = tok.begin();
        beg++; // skip id
        starts[i] = State(std::atoi(beg->c_str()));
        goal_locations[i].emplace_back(std::atoi(beg->c_str()));
    }

    myfile.close();
    return true;
}


bool BasicSystem::run()
{
    bool sol = pbs.run(starts, goal_locations, time_limit, vector<Path>(), PriorityGraph());
    pbs.save_results(outfile, std::to_string(num_of_drives) + "," + std::to_string(seed));
    pbs.best_node->priorities.save_as_digraph("goal_node.gv");
    return sol;
}
*/

bool BasicSystem::load_locations()
{
    string fname = G.map_name + "_rotation=" + std::to_string(consider_rotation) +
                   "_" + std::to_string(num_of_drives) + ".agents";
    std::ifstream myfile(fname.c_str());
    if (!myfile.is_open())
        return false;

    string line;
    getline(myfile, line);
    boost::char_separator<char> sep(",");

    if (atoi(line.c_str()) != num_of_drives)
    {
        cout << "The agent file does not match the settings." << endl;
        exit(-1);
    }
    for (int k = 0; k < num_of_drives; k++)
    {
        getline(myfile, line);
        boost::tokenizer<boost::char_separator<char>> tok(line, sep);
        boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();
        // starts
        int start_loc = atoi((*beg).c_str());
        beg++;
        int start_orient = atoi((*beg).c_str());
        beg++;
        starts[k] = State(start_loc, 0, start_orient);
        paths[k].emplace_back(starts[k]);
        finished_tasks[k].push_back(Task(start_loc, -1, 0, 0, k, 0));
        // goals
        int goal = atoi((*beg).c_str());
        goal_locations[k].emplace_back(goal, 0, 0);
    }
    myfile.close();
    return true;
}

void BasicSystem::update_start_locations()
{
    // std::cout << "in update start location, timestep = "<<timestep<<std::endl;
    for (int k = 0; k < num_of_drives; k++)
    {
        starts[k] = State(paths[k][timestep].location, 0,
                          paths[k][timestep].orientation,
                          paths[k][timestep].is_tasking_wait,
                          paths[k][timestep].is_rotating);
        // std::cout << "agent "<<k<<": loc="<<starts[k].location<<std::endl;
    }
}

void BasicSystem::update_paths(const std::vector<Path *> &MAPF_paths, int max_timestep = INT_MAX)
{
    for (int k = 0; k < num_of_drives; k++)
    {
        int length = min(max_timestep, (int)MAPF_paths[k]->size());
        paths[k].resize(timestep + length);
        for (int t = 0; t < length; t++)
        {
            if (MAPF_paths[k]->at(t).location < 0)
                paths[k][timestep + t] = State(
                    -starts[k].location - 1,
                    timestep + t,
                    starts[k].orientation,
                    starts[k].is_tasking_wait,
                    starts[k].is_rotating);
            else
            {
                paths[k][timestep + t] = MAPF_paths[k]->at(t);
                paths[k][timestep + t].timestep = timestep + t;
            }
        }
    }
}

void BasicSystem::update_paths(const std::vector<Path> &MAPF_paths, int max_timestep = INT_MAX)
{
    for (int k = 0; k < num_of_drives; k++)
    {
        int length = min(max_timestep, (int)MAPF_paths[k].size());
        paths[k].resize(timestep + length);
        for (int t = 0; t < length; t++)
        {
            bool prev_is_tasking_wait = paths[k][timestep + t].is_tasking_wait;
            paths[k][timestep + t] = MAPF_paths[k][t];
            paths[k][timestep + t].timestep = timestep + t;

            // Small trick of if considering waiting:
            // If the task is finished right at the `end_timestep` of previous
            // window, we manually ask the agent to start at the same timestep
            // with `is_tasking_wait` being `false` to make the low level solver
            // run. But then we need to make sure that `is_tasking_wait` in the
            // `paths` conforms to the value in the previous window.
            if (t == 0 && timestep > 0 &&
                prev_is_tasking_wait != MAPF_paths[k][t].is_tasking_wait)
                {
                    paths[k][timestep + t].is_tasking_wait = prev_is_tasking_wait;
                }
        }
    }
}

void BasicSystem::update_initial_paths(vector<Path> &initial_paths) const
{
    initial_paths.clear();
    initial_paths.resize(num_of_drives);
    for (int k = 0; k < num_of_drives; k++)
    {
        // check whether the path traverse every goal locations
        int i = (int)goal_locations[k].size() - 1;
        int j = (int)paths[k].size() - 1;
        while (i >= 0 && j >= 0)
        {
            while (j >= 0 &&
                   paths[k][j].location != goal_locations[k][i].location &&
                   paths[k][j].timestep >= goal_locations[k][i].hold_time)
                {
                    j--;
                }
            i--;
        }
        if (j < 0)
            continue;

        if ((int)paths[k].size() <= timestep + planning_window)
            continue;

        initial_paths[k].resize(paths[k].size() - timestep);
        for (int t = 0; t < (int)initial_paths[k].size(); t++)
        {
            initial_paths[k][t] = paths[k][timestep + t];
            initial_paths[k][t].timestep = t;
        }
    }
}

void BasicSystem::update_initial_constraints(list<tuple<int, int, int>> &initial_constraints) const
{
    initial_constraints.clear();
    for (int k = 0; k < num_of_drives; k++)
    {
        // TODO: Add initial constraint for manufacturing
        // Make agents hold the current location for enough timestep so other
        // agents won't affect it.
        if (this->is_tasking[k])
        {
            int start_timestep = this->starts[k].timestep;
            int curr_location = this->starts[k].location;
            initial_constraints.emplace_back(k, curr_location, start_timestep + 1);
        }

        // Add constraints for k_robust
        int prev_location = -1;
        for (int t = timestep; t > max(0, timestep - k_robust); t--)
        {
            int curr_location = paths[k][t].location;
            if (curr_location < 0)
                continue;
            else if (curr_location != prev_location)
            {
                initial_constraints.emplace_back(
                    k, curr_location, t + k_robust + 1 - timestep);
                prev_location = curr_location;
            }
        }
    }
}

bool BasicSystem::check_collisions(const vector<Path> &input_paths) const
{
    for (int a1 = 0; a1 < (int)input_paths.size(); a1++)
    {
        for (int a2 = a1 + 1; a2 < (int)input_paths.size(); a2++)
        {
            // TODO: add k-robust
            size_t min_path_length = input_paths[a1].size() < input_paths[a2].size() ? input_paths[a1].size() : input_paths[a2].size();
            for (size_t timestep = 0; timestep < min_path_length; timestep++)
            {
                int loc1 = input_paths[a1].at(timestep).location;
                int loc2 = input_paths[a2].at(timestep).location;
                if (loc1 == loc2)
                    return true;
                else if (timestep < min_path_length - 1 && loc1 == input_paths[a2].at(timestep + 1).location && loc2 == input_paths[a1].at(timestep + 1).location)
                    return true;
            }
            if ((hold_endpoints || useDummyPaths) && input_paths[a1].size() != input_paths[a2].size())
            {
                int a1_ = input_paths[a1].size() < input_paths[a2].size() ? a1 : a2;
                int a2_ = input_paths[a1].size() < input_paths[a2].size() ? a2 : a1;
                int loc1 = input_paths[a1_].back().location;
                for (size_t timestep = min_path_length; timestep < input_paths[a2_].size(); timestep++)
                {
                    int loc2 = input_paths[a2_].at(timestep).location;
                    if (loc1 == loc2)
                        return true;
                }
            }
        }
    }
    return false;
}

bool BasicSystem::congested() const
{
    if (simulation_window <= 1)
        return false;
    int wait_agents = 0;
    for (const auto &path : paths)
    {
        int t = 0;
        while (t < simulation_window && path[timestep].location == path[timestep + t].location &&
               path[timestep].orientation == path[timestep + t].orientation)
            t++;
        if (t == simulation_window)
            wait_agents++;
    }
    return wait_agents > num_of_drives / 2; // more than half of drives didn't make progress
}

// move all agents from start_timestep to end_timestep
// return a list of finished tasks
list<Task> BasicSystem::move()
{
    int start_timestep = timestep + 1;
    int end_timestep = timestep + simulation_window;
    this->queue_aborted_agt.clear();

    list<Task> new_finished_tasks; // <agent_id, location, timestep>

    for (int t = start_timestep; t <= end_timestep; t++)
    {
        for (int k = 0; k < num_of_drives; k++)
        {
            // Agents waits at its current locations if no future paths are assigned
            while ((int)paths[k].size() <= t)
            { // This should not happen?
                State final_state = paths[k].back();
                paths[k].emplace_back(final_state.location, final_state.timestep + 1, final_state.orientation);
            }
        }
    }

    for (int t = start_timestep; t <= end_timestep; t++)
    {
        for (int k = 0; k < num_of_drives; k++)
        {
            State curr = paths[k][t];

            /* int wait_times = 0; // wait time at the current location
            while (wait_times < t && paths[k][t - wait_times] != curr)
            {
                wait_times++;
            }*/

            // Update the tasking wait.
            if (curr.is_tasking_wait && !goal_locations[k].empty() &&
                std::find(this->queue_aborted_agt.begin(),
                          this->queue_aborted_agt.end(), k) ==
                          this->queue_aborted_agt.end())
                // The agent is doing task at the current location
            {
                this->is_tasking[k] = true;
                this->waited_time[k] += 1;
                if (this->screen > 0)
                {
                    cout << "t = " << t <<  ": Agent " << k << " doing task at "
                         << curr.location
                         << ", waited for " << this->waited_time[k]
                         << " timesteps, " << "should wait for "
                         << goal_locations[k].front().task_wait_time
                         << " timesteps" << endl;
                }
                // Check if unnecessary waiting exists
                if (!this->queue_mechanism && this->waited_time[k] >
                    goal_locations[k].front().task_wait_time)
                {
                    cout << "Agent " << k << " waited too long at "
                         << curr.location << endl;
                    save_results();
                    exit(-1);
                }
            }

            // Update the rotating wait.
            // Only run this if we consider rotation and the rotation time is
            // > 1 (meaning we need to wait for some additional timesteps)
            if (this->consider_rotation && curr.is_rotating &&
                this->rotation_time > 1)
            {
                this->is_rotating[k] = true;
                this->rotate_time[k] += 1;
                if (this->screen > 0)
                {
                    cout << "Agent " << k << " rotating at " << curr.location
                         << " to " << curr.orientation
                         << ", waited for " << this->rotate_time[k]
                         << " timesteps" << endl;
                }
                // Spend enough time to rotate
                if (this->rotate_time[k] >= this->rotation_time)
                {
                    this->is_rotating[k] = false;
                    this->rotate_time[k] = 0;
                }
            }

            // Remove goals if necessary
            // Note: we need a while loop because of an edge case:
            // suppose an agent have 3 tasks A, B, C with the same loc/ori, it
            // should wait in A and C for non-zero timesteps and should not
            // wait in B. In SIPP, we do not search for wait actions, so B will
            // be ignored. We simply allow the agent to finish more than 1
            // task in one timestep here to tackle this edge case (i.e. it
            // finishes A and B when it finish A, and start to wait for C in the
            // next timesteps).
            while (
                // There are still tasks in the task list
                (!hold_endpoints || paths[k].size() == t + 1) &&
                !goal_locations[k].empty() &&
                // Agent has arrived at the specified location
                curr.location == goal_locations[k].front().location &&
                // Agent has arrived at the specified orientation, if any.
                (goal_locations[k].front().orientation == -1 ||
                 curr.orientation == goal_locations[k].front().orientation) &&
                // Agent has hold enough time
                curr.timestep >= goal_locations[k].front().hold_time &&
                // the agent finished its task (waited enough time) at the goal
                this->waited_time[k] >=
                    goal_locations[k].front().task_wait_time)
            {
                Task finished_task = goal_locations[k].front();
                finished_task.agent_id = k;
                finished_task.finish_t = t;
                // Exclude parking locations
                if (!finished_task.is_parking)
                {
                    new_finished_tasks.emplace_back(finished_task);
                }
                goal_locations[k].erase(goal_locations[k].begin());

                if (this->screen > 0)
                {
                    cout << "Agent " << k << " finish task at "
                         << curr.location << " after waiting for "
                         << this->waited_time[k] << " timesteps" << endl;
                }

                this->waited_time[k] = 0;
                this->is_tasking[k] = false;

                // Queue mechanism
                // If the finished task is a workstation, abort the waiting of
                // the agents that in the queue
                if (this->queue_mechanism && this->G.types[finished_task.location] == "Workstation")
                {
                    // Workstation is free
                    this->goal_occupied[finished_task.location] = -1;

                    for (auto queue_loc : this->G.queue_locations.at(finished_task.location))
                    {
                        int queue_agent = this->goal_occupied[queue_loc];
                        // Check if the queue location is occupied by an agent
                        if (queue_agent != -1)
                        {
                            // Abort the waiting of the agent
                            this->queue_aborted_agt.emplace_back(queue_agent);
                            // Remove queue goal
                            this->goal_locations[queue_agent].erase(
                                this->goal_locations[queue_agent].begin());
                            // Remove dummy random goal
                            this->goal_locations[queue_agent].erase(
                                    this->goal_locations[queue_agent].begin());

                            // Queue location is free
                            this->goal_occupied[queue_loc] = -1;
                            this->waited_time[queue_agent] = 0;
                            this->is_tasking[queue_agent] = false;

                            if (this->screen > 0)
                            {
                                cout << "Agent " << queue_agent
                                     << " abort queue waiting at "
                                     << queue_loc << endl;
                            }
                        }
                    }
                }
            }

            // check whether the move is valid
            if (t > 0)
            {
                State prev = paths[k][t - 1];

                // Check if the state if valid
                if (prev.location < 0)
                {
                    cout << "Drive " << k << " has an invalid state at "
                         << prev << endl;
                    save_results();
                    exit(-1);
                }

                if (curr.location == prev.location)
                {
                    if (G.get_rotate_degree(prev.orientation, curr.orientation) == 2)
                    {
                        cout << "Drive " << k << " rotates 180 degrees from " << prev << " to " << curr << endl;
                        save_results();
                        exit(-1);
                    }
                }
                else if (consider_rotation)
                {
                    if (prev.orientation != curr.orientation)
                    {
                        cout << "Drive " << k << " rotates while moving from " << prev << " to " << curr << endl;
                        save_results();
                        exit(-1);
                    }
                    else if (!G.valid_move(prev.location, prev.orientation) ||
                             prev.location + G.move[prev.orientation] != curr.location)
                    {
                        cout << "Drive " << k << " jump from " << prev << " to " << curr << endl;
                        save_results();
                        exit(-1);
                    }
                }
                else
                {
                    int dir = G.get_direction(prev.location, curr.location);
                    if (dir < 0 || !G.valid_move(prev.location, dir))
                    {
                        cout << "Drive " << k << " jump from " << prev << " to " << curr << endl;
                        save_results();
                        exit(-1);
                    }
                }
            }

            // Check whether this move has conflicts with other agents
            if (G.types[curr.location] != "Magic")
            {
                for (int j = k + 1; j < num_of_drives; j++)
                {
                    for (int i = max(0, t - k_robust); i <= min(t + k_robust, end_timestep); i++)
                    {
                        if ((int)paths[j].size() <= i)
                            break;
                        if (paths[j][i].location == curr.location)
                        {
                            cout << "Drive " << k << " at " << curr << " has a conflict with drive " << j
                                 << " at " << paths[j][i] << endl;
                            save_results(); // TODO: write termination reason to files
                            exit(-1);
                        }
                    }
                }
            }
        }
    }
    return new_finished_tasks;
}

void BasicSystem::add_partial_priorities(const vector<Path> &initial_paths, PriorityGraph &initial_priorities) const
{
    list<int> low_priorities;
    list<int> high_priorities;
    for (int k = 0; k < num_of_drives; k++)
    {
        if (initial_paths[k].empty())
            low_priorities.push_back(k);
        else
            high_priorities.push_back(k);
    }

    for (auto low : low_priorities)
    {
        for (auto high : high_priorities)
            initial_priorities.add(low, high);
    }
}

void BasicSystem::save_results()
{
    if (this->save_result)
    {
        if (screen)
            std::cout << "*** Saving " << seed << " ***" << std::endl;
        clock_t t = std::clock();
        std::ofstream output;

        // settings
        output.open(outfile + "/config.txt", std::ios::out);
        output << "map: " << G.map_name << std::endl
               << "#drives: " << num_of_drives << std::endl
               << "seed: " << seed << std::endl
               << "solver: " << solver.get_name() << std::endl
               << "time_limit: " << time_limit << std::endl
               << "simulation_window: " << simulation_window << std::endl
               << "planning_window: " << planning_window << std::endl
               << "simulation_time: " << simulation_time << std::endl
               << "robust: " << k_robust << std::endl
               << "rotate: " << consider_rotation << std::endl
               << "use_dummy_paths: " << useDummyPaths << std::endl
               << "hold_endpoints: " << hold_endpoints << std::endl;

        output.close();

        // tasks
        output.open(outfile + "/tasks.txt", std::ios::out);
        output << num_of_drives << std::endl;
        for (int k = 0; k < num_of_drives; k++)
        {
            int prev = finished_tasks[k].front().location;
            for (auto task : finished_tasks[k])
            {
                output << task.location << "," << task.finish_t << ",";
                if (task.finish_t != 0)
                    output << G.heuristics.at(task.location)[prev];
                output << ";";
                prev = task.location;
            }
            for (auto goal : goal_locations[k]) // tasks that have not been finished yet
            {
                output << goal.location << ",-1,;";
            }
            output << std::endl;
        }
        output.close();

        // paths
        output.open(outfile + "/paths.txt", std::ios::out);
        output << num_of_drives << std::endl;
        for (int k = 0; k < num_of_drives; k++)
        {
            for (auto p : paths[k])
            {
                if (p.timestep <= timestep+simulation_window)
                    output << p << ";";
            }
            output << std::endl;
        }
        output.close();
        saving_time = (double)(std::clock() - t) / CLOCKS_PER_SEC;
        
        // weights
        output.open(outfile + "/map_weights.txt", std::ios::out);
        output << "***type***" << std::endl;
        for (std::string t : this->G.types)
            output << t << ",";
        output << std::endl;

        output << "***weights***" << std::endl;
        for (std::vector<double> n : this->G.weights)
        {
            for (double w : n)
            {
                output << w << ",";
            }
            output << std::endl;
        }

        if (screen)
            std::cout << "Done! (" << saving_time << " s)" << std::endl;        
    }
}

void BasicSystem::update_travel_times(unordered_map<int, double> &travel_times)
{
    if (travel_time_window <= 0)
        return;

    travel_times.clear();
    unordered_map<int, int> count;

    int t_min = max(0, timestep - travel_time_window);
    if (t_min >= timestep)
        return;
    for (auto path : paths)
    {
        int t = timestep;
        while (t >= t_min)
        {
            int loc = path[t].location;
            int dir = path[t].orientation;
            int wait = 0;
            while (t > wait && path[t - 1 - wait].location == loc && path[t - 1 - wait].orientation == dir)
                wait++;
            auto it = travel_times.find(loc);
            if (it == travel_times.end())
            {
                travel_times[loc] = wait;
                count[loc] = 1;
            }
            else
            {
                travel_times[loc] += wait;
                count[loc] += 1;
            }
            t = t - 1 - wait;
        }
    }

    for (auto it : count)
    {
        if (it.second > 1)
        {
            travel_times[it.first] /= it.second;
        }
    }
}

void BasicSystem::solve()
{
    this->LRA_called = false;
    LRAStar lra(G, solver.path_planner);
    lra.simulation_window = simulation_window;
    lra.k_robust = k_robust;
    solver.clear();
    this->n_mapf_calls++;

    // std::cout << "solve, init real goal" <<std::endl;
    // If we consider waiting time of tasks, we should ignore the first
    // goal of the agent if the agent still doing task in that goal
    // from the previous windowed MAPF run
    vector<vector<Task>> real_goal_locations;
    real_goal_locations.resize(num_of_drives);
    for (int k = 0; k < num_of_drives; k++)
    {
        int j = 0;
        // Agent is still doing task if the next goal is the same at the start
        // location
        if (this->is_tasking[k] &&
            starts[k].location == goal_locations[k].front().location &&
            (goal_locations[k].front().orientation == -1 ||
            goal_locations[k].front().orientation == starts[k].orientation))
        {
            j = 1;
        }
        for (; j < goal_locations[k].size(); j++)
        {
            real_goal_locations[k].emplace_back(goal_locations[k][j]);
        }
    }

    if (solver.get_name() == "LRA")
    {
        // predict travel time
        unordered_map<int, double> travel_times;
        update_travel_times(solver.travel_times);

        bool sol = solver.run(starts, real_goal_locations, time_limit);
        update_paths(solver.solution);
    }
    else if (solver.get_name() == "WHCA")
    {
        update_initial_constraints(solver.initial_constraints);

        bool sol = solver.run(starts, real_goal_locations, time_limit);
        if (sol)
        {
            update_paths(solver.solution);
        }
        else
        {
            lra.resolve_conflicts(solver.solution);
            update_paths(lra.solution);
            this->n_rule_based_calls++;
        }
    }
    else // PBS or ECBS
    {
        // PriorityGraph initial_priorities;
        // std::cout << "solve, update_initial_constraints" <<std::endl;
        update_initial_constraints(solver.initial_constraints);

        // solve
        if (hold_endpoints || useDummyPaths)
        {
            if (solver.consider_task_wait)
            {
                cout << "Task wait is not supported for `hold_endpoints` and "
                        "`useDummyPaths`"
                     << endl;
                exit(-1);
            }
            vector<State> new_starts;
            vector<vector<Task>> new_goal_locations;
            for (int i : new_agents)
            {
                new_starts.emplace_back(starts[i]);
                new_goal_locations.emplace_back(goal_locations[i]);
            }
            vector<Path> planned_paths(num_of_drives);
            solver.initial_rt.clear();
            auto p = new_agents.begin();
            for (int i = 0; i < num_of_drives; i++)
            {
                planned_paths[i].resize(paths[i].size() - timestep);
                for (int t = 0; t < (int)planned_paths[i].size(); t++)
                {
                    planned_paths[i][t] = paths[i][timestep + t];
                    planned_paths[i][t].timestep = t;
                }
                if (p == new_agents.end() || *p != i)
                {
                    solver.initial_rt.insertPath2CT(planned_paths[i]);
                }
                else
                    ++p;
            }
            if (!new_agents.empty())
            {
                bool sol;
                if (timestep == 0)
                {
                    sol = solver.run(
                        new_starts, new_goal_locations, 20 * time_limit);
                }
                else
                {
                    sol = solver.run(
                        new_starts, new_goal_locations, time_limit);
                }
                if (sol)
                {
                    auto pt = solver.solution.begin();
                    for (int i : new_agents)
                    {
                        planned_paths[i] = *pt;
                        ++pt;
                    }
                    if (check_collisions(planned_paths))
                    {
                        cout << "COLLISIONS!" << endl;
                        exit(-1);
                    }
                }
                else
                {
                    sol = solve_by_WHCA(
                        planned_paths, new_starts, new_goal_locations);
                    assert(sol);
                }
            }
            // lra.resolve_conflicts(planned_paths, k_robust);
            update_paths(planned_paths);
        }
        else
        {
            // std::cout << "solve, solver.run" <<std::endl;
            bool sol = solver.run(
                starts, real_goal_locations, time_limit, waited_time);
            if (sol)
            {
                // std::cout << "solve, has sol" <<std::endl;
                if (log)
                    solver.save_constraints_in_goal_node(outfile + "/goal_nodes/" + std::to_string(timestep) + ".gv");
                update_paths(solver.solution);
            }
            else
            {
                // std::cout << "solve, no sol" <<std::endl;
                lra.resolve_conflicts(solver.solution);
                update_paths(lra.solution);
                this->n_rule_based_calls++;
            }
        }
        if (log)
            solver.save_search_tree(outfile + "/search_trees/" + std::to_string(timestep) + ".gv");
    }

    if (this->save_solver)
    {
        solver.save_results(outfile + "/solver.csv", std::to_string(timestep) + "," + std::to_string(num_of_drives) + "," + std::to_string(seed));
    }
}

bool BasicSystem::solve_by_WHCA(vector<Path> &planned_paths,
                                const vector<State> &new_starts,
                                const vector<vector<Task>> &new_goal_locations)
{
    WHCAStar whca(G, solver.path_planner);
    whca.k_robust = k_robust;
    whca.window = INT_MAX;
    whca.hold_endpoints = hold_endpoints || useDummyPaths;
    whca.screen = screen;
    whca.initial_rt.hold_endpoints = true;
    whca.initial_rt.map_size = G.size();
    whca.initial_rt.k_robust = k_robust;
    whca.initial_rt.window = INT_MAX;
    whca.initial_rt.copy(solver.initial_rt);
    whca.initial_solution.resize(new_starts.size());
    if (whca.hold_endpoints)
    {
        if (timestep == 0)
        {
            for (int i = 0; i < (int)new_starts.size(); i++)
                whca.initial_solution[i].emplace_back(starts[i]); // hold initial location
        }
        else
        {
            whca.initial_solution.clear();
            for (auto agent : new_agents)
                whca.initial_solution.emplace_back(planned_paths[agent]); // hold old paths
        }
    }
    bool sol = false;
    if (timestep == 0)
        sol = whca.run(new_starts, new_goal_locations, 20 * time_limit);
    else
        sol = whca.run(new_starts, new_goal_locations, time_limit);
    whca.save_results(outfile + "/solver.csv", std::to_string(timestep) + "," + std::to_string(num_of_drives) + "," + std::to_string(seed));
    if (sol)
    {
        auto pt = whca.solution.begin();
        for (int i : new_agents)
        {
            planned_paths[i] = *pt;
            ++pt;
        }
    }
    whca.clear();
    return sol;
}

void BasicSystem::initialize_solvers()
{
    solver.k_robust = k_robust;
    solver.window = planning_window;
    solver.hold_endpoints = hold_endpoints || useDummyPaths;
    solver.screen = screen;

    solver.initial_rt.hold_endpoints = true;
    solver.initial_rt.map_size = G.size();
    solver.initial_rt.k_robust = k_robust;
    solver.initial_rt.window = INT_MAX;
}

bool BasicSystem::load_records()
{
    boost::char_separator<char> sep1(";");
    boost::char_separator<char> sep2(",");
    string line;

    // load paths
    std::ifstream myfile(outfile + "/paths.txt");

    if (!myfile.is_open()){
        std::cout << "not find log file:" <<outfile + "/paths.txt"<<std::endl;
        return false;
    }
        

    timestep = INT_MAX;
    getline(myfile, line);
    if (atoi(line.c_str()) != num_of_drives)
    {
        cout << "The path file does not match the settings." << endl;
        exit(-1);
    }
    for (int k = 0; k < num_of_drives; k++)
    {
        getline(myfile, line);
        boost::tokenizer<boost::char_separator<char>> tok1(line, sep1);
        for (auto task : tok1)
        {
            boost::tokenizer<boost::char_separator<char>> tok2(task, sep2);
            boost::tokenizer<boost::char_separator<char>>::iterator beg = tok2.begin();
            int loc = atoi((*beg).c_str());
            beg++;
            int orientation = atoi((*beg).c_str());
            beg++;
            int time = atoi((*beg).c_str());
            paths[k].emplace_back(loc, time, orientation);
        }
        timestep = min(timestep, paths[k].back().timestep);
        // std::cout << "timestep for k="<<k<<", "<<timestep<<std::endl;
    }
    myfile.close();

    // pick the timestep
    // timestep = int((timestep - 1) / simulation_window) * simulation_window; // int((timestep - 1) / simulation_window) * simulation_window;

    // load tasks
    myfile.open(outfile + "/tasks.txt");
    if (!myfile.is_open())
        return false;

    getline(myfile, line);
    if (atoi(line.c_str()) != num_of_drives)
    {
        cout << "The task file does not match the settings." << endl;
        exit(-1);
    }
    for (int k = 0; k < num_of_drives; k++)
    {
        getline(myfile, line);
        boost::tokenizer<boost::char_separator<char>> tok1(line, sep1);
        for (auto task : tok1)
        {
            boost::tokenizer<boost::char_separator<char>> tok2(task, sep2);
            boost::tokenizer<boost::char_separator<char>>::iterator beg = tok2.begin();
            int loc = atoi((*beg).c_str());
            beg++;
            int time = atoi((*beg).c_str());
            if (time >= 0 && time <= timestep)
            {
                finished_tasks[k].emplace_back(Task(loc, -1, 0, 0, k, time));
                timestep = max(timestep, time);
            }
            else
            {
                goal_locations[k].emplace_back(loc, 0, 0);
            }
        }
        std::cout << "agent "<<k<<" reload goals"<<std::endl;
        for(int j=0; j<goal_locations[k].size(); ++j){
            std::cout << goal_locations[k][j].location <<", ";
        }
        std::cout << std::endl;
    }
    myfile.close();
    // exit(1);
    return true;
}

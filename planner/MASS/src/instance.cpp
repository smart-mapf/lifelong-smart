#include "instance.h"

int RANDOM_WALK_STEPS = 100000;

Instance::Instance(shared_ptr<Graph> graph, shared_ptr<RobotMotion> bot_motion,
                   json mapf_instance, bool use_partial_expansion,
                   int used_sps_solver, int screen, double simulation_window)
    : graph(graph),
      bot_motion(bot_motion),
      use_pe(use_partial_expansion),
      use_sps_type(used_sps_solver),
      screen(screen),
      simulation_window(simulation_window) {
    this->num_of_agents = mapf_instance["starts"].size();
    GetRawReservationTable();
    bool succ = loadAgents(mapf_instance);
    if (!succ) {
        spdlog::error("Failed to load agents.");
        if (num_of_agents > 0) {
            generateRandomAgents();
            // saveAgents();
        }
    }
}

Instance::~Instance() {
    // Destructor to clean up resources if needed
    // graph = nullptr;
    // task_assigner = nullptr;
    // bot_motion = nullptr;
    // agents.clear();
}

void Instance::GetRawReservationTable() {
    raw_rv_tbl.resize(this->graph->map_size);
    TimeInterval blocked_interval;
    blocked_interval.t_min = 0;
    blocked_interval.t_max = INF;
    blocked_interval.agent_id = -1;
    for (int i = 0; i < this->graph->map_size; i++) {
        if (this->graph->my_map[i]) {
            raw_rv_tbl[i].push_back(blocked_interval);
        }
    }
}

void Instance::GetAgents(std::vector<Agent>& agents_list) {
    agents_list = agents;
}

void Instance::generateRandomAgents() {
    cout << "Generate " << num_of_agents << " random start and goal locations "
         << endl;
    vector<bool> starts(this->graph->map_size, false);
    vector<bool> goals(this->graph->map_size, false);
    agents.resize(num_of_agents);
    int task_id = 0;

    // Non-warehouse scenario
    if (this->graph->workstations.size() == 0)  // Generate agents randomly
    {
        if (screen > 0)
            spdlog::info("Generating {} random agents...", num_of_agents);
        // Choose random start locations
        int k = 0;
        while (k < num_of_agents) {
            int x = rand() % this->graph->num_of_rows,
                y = rand() % this->graph->num_of_cols;
            int start = this->graph->linearizeCoordinate(x, y);
            if (this->graph->my_map[start] || starts[start])
                continue;

            // update start
            starts[start] = true;

            // Find goals, 2 for each agent, for testing purpose
            vector<Task> tasks;
            for (int i = 0; i < 2; i++) {
                int goal = this->graph->randomWalk(start, RANDOM_WALK_STEPS);
                while (goals[goal])
                    goal = this->graph->randomWalk(goal, 1);

                // update goal
                goals[goal] = true;
                tasks.push_back(Task(task_id, goal, orient::None));
                task_id++;
            }

            agents[k] = Agent(start, orient::East, tasks, bot_motion);
            k++;
        }
    } else  // Generate agents for warehouse scenario
    {
        spdlog::info("Generating {} random agents for warehouse scenario...",
                     num_of_agents);
        // Choose random start locations
        for (int k = 0; k < num_of_agents; k++) {
            int start = this->graph->sampleFreeLocation();
            while (starts[start]) {
                start = this->graph->sampleFreeLocation();
            }
            starts[start] = true;

            vector<Task> tasks;
            for (int i = 0; i < 2; i++) {
                // Choose a random goal from the warehouse task locations
                int goal = this->graph->sampleWarehouseTaskLoc();
                while (goals[goal] || goal == start) {
                    goal = this->graph->sampleWarehouseTaskLoc();
                }
                goals[goal] = true;

                // spdlog::info(
                //     "Agent {}: Start location: {}, Goal location: {}",
                //     k, start, goal);

                tasks.push_back(Task(task_id, goal, orient::None));
                task_id++;
            }
            agents[k] = Agent(start, orient::East, tasks, bot_motion);
        }

        // Print goal locations of the agent
        if (screen > 0) {
            spdlog::info("MAPF instances:");
            for (int i = 0; i < num_of_agents; i++) {
                cout << "Agent " << i << ": Start location: ("
                     << graph->getCoordinate(agents[i].start_location).first
                     << ", "
                     << graph->getCoordinate(agents[i].start_location).second
                     << ") => ";
                for (const auto& task : agents[i].goal_locations) {
                    cout << "(" << graph->getCoordinate(task.loc).first << ", "
                         << graph->getCoordinate(task.loc).second << ") -> ";
                }
                cout << endl;
            }
        }
    }
}

bool Instance::loadAgents(json mapf_instance) {
    json starts_json = mapf_instance.at("starts");
    json goals_json = mapf_instance.at("goals");
    this->num_of_agents = starts_json.size();
    this->agents.resize(this->num_of_agents);

    for (int i = 0; i < this->num_of_agents; i++) {
        int start_loc = starts_json[i].at("location");
        int start_ori = starts_json[i].at("orientation");
        vector<Task> tasks;
        for (const auto& goal : goals_json[i]) {
            int task_loc = goal.at("location");
            int task_id = goal.at("id");
            tasks.push_back(Task(task_id, task_loc, orient::None));
        }
        this->agents[i] =
            Agent(start_loc, this->graph->ORI_RHCR_TO_MASS.at(start_ori), tasks,
                  bot_motion);
    }
    return true;
}

void Instance::saveAgents(string filename) const {
    ofstream myfile(filename);
    spdlog::info("Saving agents to {}", filename);
    if (!myfile.is_open()) {
        // cout << "Fail to save the agents to " << filename << endl;
        spdlog::error("Fail to save the agents to {}", filename);
        return;
    }
    myfile << num_of_agents << endl;

    for (int i = 0; i < num_of_agents; i++) {
        Agent curr_agent = agents[i];
        // Write the start location and orientation as the first entry of each
        // agent
        myfile << curr_agent.start_location << ","
               << static_cast<int>(curr_agent.start_o) << ";";
        for (const auto& task : curr_agent.goal_locations) {
            myfile << task.loc << "," << static_cast<int>(task.ori) << ";";
        }
        myfile << endl;
    }
    myfile.close();
}

#include "ExecutionManager.h"

ExecutionManager::ExecutionManager(
    const boost::program_options::variables_map vm)
    : _vm(vm),
      output_filename(vm["output_file"].as<string>()),
      save_stats(vm["save_stats"].as<bool>()),
      screen(vm["screen"].as<int>()),
      port(vm["port_number"].as<int>()),
      total_sim_step_tick(vm["total_sim_step_tick"].as<int>()),
      ticks_per_second(vm["ticks_per_second"].as<int>()),
      sim_window_tick(vm["sim_window_tick"].as<int>()),
      look_ahead_tick(vm["look_ahead_tick"].as<int>()),
      // Hacky way to make sure the simulation is frozen at the beginning
      prev_invoke_planner_tick(-vm["look_ahead_tick"].as<int>()),
      seed(vm["seed"].as<int>()),
      planner_invoke_policy(vm["planner_invoke_policy"].as<string>()),
      adg(make_shared<ADG>(vm["num_robots"].as<int>(), vm["screen"].as<int>(),
                           vm["look_ahead_dist"].as<int>())),
      numRobots(vm["num_robots"].as<int>()),
      parser(PlanParser(vm["screen"].as<int>())),
      tick_per_robot(vector<int>(vm["num_robots"].as<int>(), 0)),
      task_assigner_type(vm["task_assigner_type"].as<string>()) {
    spdlog::info("Invoke policy: {}", planner_invoke_policy);
    spdlog::info("Look ahead tick: {}", look_ahead_tick);
    spdlog::info("Look ahead dist: {}", adg->getLookAheadDist());
    spdlog::info("Simulation window tick: {}", sim_window_tick);
}

bool ExecutionManager::stopSimulation() {
    return this->congested_sim || this->simulationFinished();
}

bool ExecutionManager::plannerNeverInvoked() {
    return prev_invoke_planner_tick < 0;
}
bool ExecutionManager::simulationFinished() {
    return this->getCurrSimStep() >= this->total_sim_step_tick;
}

bool ExecutionManager::simTickElapsedFromLastInvoke(int ticks) {
    return this->getCurrSimStep() - this->prev_invoke_planner_tick >= ticks;
}

int ExecutionManager::getCurrSimStep() {
    return *min_element(tick_per_robot.begin(), tick_per_robot.end());
}

void ExecutionManager::setupBackupPlanner() {
    spdlog::info("Setting up backup planner...");
    // Setup graph.
    this->G.screen = this->screen;
    this->G.hold_endpoints = false;
    this->G.useDummyPaths = false;
    // Backup planner is based on PIBT. Do not consider rotation.
    // TODO: Make heuristic table rotational compatible
    this->G.consider_rotation = false;
    // this->G.rotation_time = this->_vm["rotation_time"].as<int>();
    // Grid type
    string grid_type = this->_vm["grid_type"].as<std::string>();
    if (!convert_G_type.count(grid_type)) {
        spdlog::error("Grid type {} does not exist!", grid_type);
        exit(-1);
    }
    this->G.set_grid_type(convert_G_type.at(grid_type));

    // TODO: Add support for one_bot_per_aisle grid type
    if (this->G.get_grid_type() == SMARTGridType::ONE_BOT_PER_AISLE) {
        spdlog::error("Warning: ONE_BOT_PER_AISLE grid type is not fully "
                      "supported in backup planner module yet.");
        exit(-1);
    }

    std::ifstream i(this->_vm["map"].as<std::string>());
    json map_json;
    i >> map_json;
    if (!G.load_map_from_jsonstr(map_json.dump(4), 1.0, 1.0)) {
        return;
    }

    // Setup single agent path planner
    this->setupSingleAgentPlanner();

    // Setup heuristic table. We are computing heuristics here and in the
    // planner, which is redundant. Consider optimizing this later.
    this->setupHeuristicTable();

    // Setup backup MAPF planner
    string solver_name = this->_vm["backup_planner"].as<string>();
    spdlog::info("Backup planner: {}", solver_name);
    if (solver_name == "PIBT") {
        this->backup_planner = make_shared<PIBT>(
            this->G, *this->path_planner, this->heuristic_table, this->_vm);
    } else if (solver_name == "GuidedPIBT") {
        this->backup_planner = make_shared<GuidedPIBT>(
            this->G, *this->path_planner, this->heuristic_table, this->_vm);
    } else if (solver_name == "LRAStar") {
        this->backup_planner = make_shared<LRAStar>(
            this->G, *this->path_planner, this->heuristic_table, this->_vm);
    } else {
        spdlog::error("Backup solver {} does not exist!", solver_name);
        exit(-1);
    }

    // Set initialized flag
    this->backup_planner->set_initialized(true);
}

void ExecutionManager::setupSingleAgentPlanner() {
    // Setup backup single agent planner
    string single_solver_name =
        this->_vm["backup_single_agent_solver"].as<string>();
    spdlog::info("Setting up single-agent planner: {}...", single_solver_name);
    if (single_solver_name == "ASTAR") {
        path_planner = make_shared<StateTimeAStar>();
    } else if (single_solver_name == "SIPP") {
        path_planner = make_shared<SIPP>();
    } else {
        spdlog::error("Backup single-agent solver {} does not exist!",
                      single_solver_name);
        exit(-1);
    }

    // Set parameters for the path planner
    path_planner->rotation_time = this->_vm["rotation_time"].as<int>();
}

void ExecutionManager::setupHeuristicTable() {
    string h_type = this->_vm["heuristic_type"].as<string>();
    spdlog::info("Setting up heuristic table: {}...", h_type);
    // string log_dir = this->_vm["output"].as<string>();
    string log_dir = ".";
    int seed = this->_vm["seed"].as<int>();
    bool save_heuristics_table = this->_vm["save_heuristics_table"].as<bool>();
    if (h_type == "basic") {
        this->heuristic_table = make_shared<BasicHeuristicTable>(
            G, G.free_locations, seed, log_dir, save_heuristics_table);
    } else if (h_type == "lazy") {
        this->heuristic_table = make_shared<LazyHeuristicTable>(
            G, G.free_locations, seed, log_dir, save_heuristics_table);
    } else if (h_type == "landmark") {
        this->heuristic_table = make_shared<LandmarkHeuristicTable>(
            G, G.free_locations, seed, log_dir, save_heuristics_table,
            this->_vm["num_landmarks"].as<int>(),
            this->_vm["landmark_selection"].as<string>());
    } else {
        spdlog::error("Heuristic table {} does not exist!", h_type);
        exit(-1);
    }
}

void ExecutionManager::setupTaskAssigner() {
    spdlog::info("Setting up task assigner...");
    string task_file = this->_vm["task_file"].as<string>();
    if (this->task_assigner_type == "windowed") {
        this->task_assigner = make_shared<WindowedTaskAssigner>(
            this->G, this->screen, this->_vm["sim_window_timestep"].as<int>(),
            this->numRobots, this->seed, task_file);
    } else if (this->task_assigner_type == "distinct_one_goal") {
        this->task_assigner = make_shared<DistinctOneGoalTaskAssigner>(
            this->G, this->screen, this->numRobots, this->seed, task_file);
    } else if (this->task_assigner_type == "one_goal") {
        this->task_assigner = make_shared<OneGoalTaskAssigner>(
            this->G, this->screen, this->numRobots, this->seed, task_file);
    } else {
        spdlog::error("Task assigner type {} does not exist!",
                      this->task_assigner_type);
        exit(-1);
    }
}

void ExecutionManager::saveStats() {
    auto end = std::chrono::steady_clock::now();
    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(
        end - this->start_time);
    this->overall_runtime = elapsed_seconds.count();

    // Compute throughput as the number of finished tasks per sim second
    int total_finished_tasks = adg->getNumFinishedTasks();
    int total_finished_backup_tasks = adg->getNumFinishedBackupTasks();
    int sim_seconds = total_sim_step_tick / ticks_per_second;
    double throughput = static_cast<double>(total_finished_tasks -
                                            total_finished_backup_tasks) /
                        sim_seconds;
    json result = {{"total_finished_tasks", total_finished_tasks},
                   {"total_finished_backup_tasks", total_finished_backup_tasks},
                   {"throughput", throughput},
                   {"success", true},
                   {"cpu_runtime", this->overall_runtime},
                   {"congested", this->congested_sim},
                   {"tasks_finished_timestep", this->tasks_finished_per_sec},
                   {"planner_invoke_ticks", this->planner_invoke_ticks},
                   {"n_planner_invokes", this->planner_invoke_ticks.size()}};

    // Unwrap and add planner stats to results
    json planner_stats_json = json::parse(this->planner_stats);
    for (auto& [key, value] : planner_stats_json.items()) {
        result[key] = value;
    }

    // Add ADG statistics
    json adg_stats = adg->getADGStats();
    for (auto& [key, value] : adg_stats.items()) {
        result[key] = value;
    }

    // Print some key statistics to console
    spdlog::info("Simulation count: {}", this->tick_per_robot[0]);
    // spdlog::info("Number of actual finished tasks: {}",
    //              this->adg->countFinishedTasks());
    // spdlog::info("Average num of rotation per robot: {:.2f}",
    //              this->adg->avgRotation());
    spdlog::info("Number of finished tasks: {}",
                 this->adg->getNumFinishedTasks());
    spdlog::info("Number of finished backup tasks: {}",
                 this->adg->getNumFinishedBackupTasks());
    spdlog::info("Overall runtime: {:.2f} seconds", this->overall_runtime);

    vector<string> key_stats = {
        "n_rule_based_calls", "mean_avg_rotation", "mean_avg_move",
        "avg_total_actions",  "n_planner_invokes",
    };
    for (const auto& stat : key_stats) {
        if (result.contains(stat)) {
            spdlog::info("{}: {}", stat, result[stat].dump());
        }
    }

    // Write the statistics to the output file
    if (this->save_stats) {
        ofstream stats(output_filename);
        stats << result.dump(4);  // Pretty print with 4 spaces

        spdlog::info("Statistics written to {}", output_filename);
    }
}

void ExecutionManager::freezeSimulationIfNecessary(string RobotID) {
    int robot_id = this->adg->startIndexToRobotID[RobotID];
    // Logic 1: freeze the simulation if at least one robot has no actions
    // if (this->adg->getNumUnfinishedActions(robot_id) <= 0) {
    //     this->freeze_simulation = true;
    //     if (this->screen > 0) {
    //         cout << "Robot " << robot_id
    //                   << " requests to freeze the simulation!" << endl;
    //     }
    // }

    // Logic 2: freeze the simulation if simulation tick has passed
    // more than `look_ahead_tick` than `prev_invoke_planner_tick`, or when
    // planner has never been invoked. Basically, if the planner does not
    // return in time. This is aim at synchronizing the simulation time with
    // world clock time
    int sim_step = this->getCurrSimStep();
    // int sim_step = this->time_step_tick;
    // spdlog::info("Checking freeze simulation at sim step {}, "
    //              "prev_invoke_planner_tick: {}, sim_window_tick: {}",
    //              sim_step, this->prev_invoke_planner_tick,
    //              this->sim_window_tick);
    if (sim_step == 0 && this->plannerNeverInvoked()) {
        this->freeze_simulation = true;
        if (this->screen > 0) {
            spdlog::info(
                "Robot {} requests to freeze the simulation at the first tick!",
                robot_id);
        }
    } else if (this->simTickElapsedFromLastInvoke(this->look_ahead_tick) &&
               this->planner_running && !this->simulationFinished()) {
        this->freeze_simulation = true;
        if (this->screen > 0) {
            spdlog::info(
                "Robot {} requests to freeze the simulation at sim step {} "
                "due to planner not return in time (synchronize)!",
                robot_id, sim_step);
        }
    }
}

string ExecutionManager::getRobotsLocation() {
    // Return message as a JSON string
    json result_message = {};

    if (this->screen > 0) {
        spdlog::info("Get robot location query received!");
    }

    this->curr_robot_states = this->adg->computeCommitCut();

    vector<tuple<double, double, int>> robots_location;

    // If no robot has initialized yet, return empty locations
    if (this->curr_robot_states.empty()) {
        result_message["robots_location"] = {};
        result_message["new_finished_tasks"] = {};
        result_message["initialized"] = false;
        return result_message.dump();
    }

    // Sanity check for number of robots
    if (this->curr_robot_states.size() !=
        static_cast<size_t>(this->numRobots)) {
        spdlog::warn("Warning: current robot states size {} does not match num "
                     "robots {}!",
                     this->curr_robot_states.size(), this->numRobots);
    }

    // Get the current locations of all robots
    for (auto& robot : this->curr_robot_states) {
        if (this->flipped_coord) {
            robots_location.emplace_back(make_tuple(
                robot.position.second, robot.position.first, robot.orient));
        } else {
            robots_location.emplace_back(make_tuple(
                robot.position.first, robot.position.second, robot.orient));
        }
    }

    set<int> new_finished_tasks = this->adg->updateFinishedTasks();

    this->tasks_finished_per_sec.push_back(
        make_tuple(new_finished_tasks.size(),
                   this->getCurrSimStep() / this->ticks_per_second));

    // result_message["robots_location"] = robots_location;
    // result_message["new_finished_tasks"] = new_finished_tasks;
    result_message["initialized"] = true;

    // Update the start and goal locations
    this->task_assigner->updateStartsAndGoals(robots_location,
                                              new_finished_tasks);
    result_message["mapf_instance"] =
        this->task_assigner->getMAPFInstanceJSON();

    // Write mapf_instance to a file
    if (this->screen > 1) {
        int tick = this->getCurrSimStep();
        // string filename = "mapf_instance_step_" + to_string(tick) + ".json";
        string filename = "mapf_instance.json";
        std::ofstream mapf_instance_file(filename);
        mapf_instance_file << result_message["mapf_instance"].dump(4);
    }

    return result_message.dump();
}

// Add a new MAPF plan to the ADG. Use backup planner if necessary
// new_plan: a vector of paths, each path is a vector of (row, col, t, task_id)
// Raw plan --> points --> Steps --> Actions
void ExecutionManager::addNewPlan(string& new_plan_json_str) {
    json new_plan_json = json::parse(new_plan_json_str);

    // Sanity check for the necessary fields
    if (!new_plan_json.contains("plan")) {
        spdlog::error("addNewPlan: No `plan` is found!");
        exit(-1);
    }

    if (!new_plan_json.contains("success")) {
        spdlog::warn("addNewPlan: No success is found! Assume failure.");
    }

    vector<vector<UserState>> new_plan;
    new_plan = new_plan_json["plan"].get<vector<vector<UserState>>>();

    bool congested = new_plan_json.value("congested", false);
    bool success = new_plan_json.value("success", false);

    // Planner fails. Use the backup planner.
    // success = false;
    if (!success) {
        auto starts = this->task_assigner->getStarts();
        auto goal_locations = this->task_assigner->getGoalLocations();

        // Obtain the guide paths
        vector<Path> guide_paths = this->convertPlanToGuidePaths(new_plan);

        // Plan using the backup planner
        if (screen > 0)
            spdlog::info("Planner fails, using backup planner");
        this->backup_planner->clear();
        this->backup_planner->run(starts, goal_locations, guide_paths);
        new_plan = this->backup_planner->convert_path_to_smart(goal_locations);
        congested = this->backup_planner->congested();
    } else {
        // Planner is successful. Add additional sanity check for collisions.
    }

    // TODO: consider backup planner for the following entries.
    // Store stats, if available
    if (new_plan_json.contains("stats")) {
        this->planner_stats = new_plan_json["stats"];
    }

    // Update backup tasks, if available
    this->adg->backup_tasks = this->task_assigner->getBackupTasks();

    if (congested) {
        // Stop the server early.
        // NOTE: We cannot call closeServer directly because we need to ensure
        // the clients (robots) are closed. So we set a flag to let the robots
        // know the simulation should be stopped and the robots will call
        // closeServer.
        spdlog::info("Congested simulation detected, stopping the simulation!");
        this->congested_sim = true;
    }

    // Convert raw plan to points
    vector<vector<Step>> steps;
    assert(new_plan.size() == this->numRobots);
    for (int agent_id = 0; agent_id < this->numRobots; agent_id++) {
        vector<Point> points;
        vector<Step> curr_steps;
        for (auto& step : new_plan[agent_id]) {
            points.emplace_back(
                Point(get<0>(step), get<1>(step), get<2>(step), get<3>(step)));
        }

        // Convert points to steps, which adds rotational states to the plan
        // returned by the MAPF planner if needed
        this->parser.AgentPathToSteps(points, curr_steps,
                                      this->curr_robot_states[agent_id].orient,
                                      agent_id);
        steps.push_back(curr_steps);
    }

    // Convert steps to actions, each two consecutive steps will be transformed
    // to at most two actions.
    vector<vector<Action>> actions;
    actions = this->parser.StepsToActions(steps, this->flipped_coord);

    this->adg->addMAPFPlan(actions);

    // Defreeze the simulation if it was frozen
    if (this->freeze_simulation) {
        this->freeze_simulation = false;
        if (this->screen > 0) {
            // cout << "Simulation is de-frozen after adding a new plan!"
            //           << endl;
            spdlog::info("Simulation is de-frozen after adding a new plan!");
        }
    }

    // Planner stop running
    this->planner_running = false;
}

// Obtain the guide paths. Guide paths are assumed to be
// non-collision-free solutions from the planner.
vector<Path> ExecutionManager::convertPlanToGuidePaths(
    const vector<vector<UserState>>& plan) {
    vector<Path> guide_paths(this->numRobots);
    for (int k = 0; k < this->numRobots; k++) {
        Path path;
        auto raw_path = plan[k];
        for (auto& step : raw_path) {
            int row = get<0>(step);
            int col = get<1>(step);
            int loc = this->G.getCellId(row, col);
            int t = static_cast<int>(get<2>(step));
            // For some algos, remove waiting from the path as guide paths are
            // meant to be spatial
            if (this->backup_planner->get_name() == "GuidedPIBT" && t > 0 &&
                path.back().location == loc) {
                continue;
            }
            path.push_back(State(loc, t));
        }
        guide_paths[k] = path;
    }
    return guide_paths;
}

string ExecutionManager::actionFinished(string& robot_id_str, int node_ID) {
    if (not this->adg->initialized) {
        spdlog::error(
            "ExecutionManager::actionFinished: ADG is not initialized");
        exit(-1);
    }
    int agent_id = this->adg->startIndexToRobotID[robot_id_str];
    bool status_update = this->adg->updateFinishedNode(agent_id, node_ID);

    return "None";
}

void ExecutionManager::init(string RobotID, tuple<int, int> init_loc) {
    // Initialize the backup planner if it has not been initialized
    if (this->backup_planner == nullptr ||
        !this->backup_planner->is_initialized()) {
        this->setupBackupPlanner();
    }

    // Initialize the task assigner if it has not been initialized
    if (this->task_assigner == nullptr) {
        this->setupTaskAssigner();
    }

    // Inform ADG with the current robot's initial location
    this->adg->createRobotIDToStartIndexMaps(RobotID, init_loc);
}

SIM_PLAN ExecutionManager::obtainActionsFromADG(string RobotID) {
    if (not this->adg->initialized or not this->adg->get_initial_plan) {
        return {};
    }

    int Robot_ID = this->adg->startIndexToRobotID[RobotID];

    return this->adg->getPlan(Robot_ID);
}

void ExecutionManager::updateSimStep(string RobotID) {
    int Robot_ID = this->adg->startIndexToRobotID[RobotID];
    this->tick_per_robot[Robot_ID]++;
}

bool ExecutionManager::invokePlanner() {
    // Should take the min of the ticks of all the robots
    int sim_step = this->getCurrSimStep();
    bool invoke = false;
    int invoke_by = -1;

    // Default: invoke planner every sim_window_tick
    if (this->planner_invoke_policy == "default") {
        invoke = (sim_step == 0 ||
                  this->simTickElapsedFromLastInvoke(this->sim_window_tick)) &&
                 !this->simulationFinished() &&
                 sim_step != this->prev_invoke_planner_tick;
    }
    // No action: invoke planner when the number of actions left for a robot
    // is less than look_ahead_dist, and at least 1 has passed
    // since last invocation.
    else if (this->planner_invoke_policy == "no_action") {
        for (int agent_id = 0; agent_id < this->numRobots; agent_id++) {
            if (this->adg->getNumUnfinishedActions(agent_id) <=
                this->adg->getLookAheadDist()) {
                invoke = true;
                invoke_by = agent_id;
                break;
            }
        }

        // Ensure we have not reached the total simulation step tick
        invoke &= !this->simulationFinished();

        // Ensure at least 1 has passed since last invocation
        if (invoke && !this->plannerNeverInvoked() &&
            !this->simTickElapsedFromLastInvoke(1)) {
            if (this->screen > 1) {
                spdlog::info(
                    "Timestep {}: Attempt to invoke by {}, but skipped to "
                    "ensure {} has passed since last invocation at {}.",
                    sim_step, invoke_by, 1, this->prev_invoke_planner_tick);
            }
            invoke = false;
            invoke_by = -1;
        }
    } else {
        spdlog::error("Unknown planner invoke policy: {}",
                      this->planner_invoke_policy);
        exit(-1);
    }

    // First invoke, start record runtime
    if (invoke && sim_step == 0) {
        this->start_time = std::chrono::steady_clock::now();
        spdlog::info("Start time recorded at sim step 0.");
    }

    // Print the unfinished actions for each robot
    if (this->screen > 0) {
        // cout << "#####################" << endl;
        // cout << "Checking if planner should be invoked at sim step: "
        //      << sim_step << ", invoke: " << invoke << endl;
        // spdlog::info("Checking if planner should be invoked at sim step: {},
        // "
        //              "invoke: {}",
        //              sim_step, invoke);
        // cout << "Unfinished actions for each robot at sim step "
        //           << sim_step << ":" << endl;
        // for (int agent_id = 0; agent_id < this->numRobots; agent_id++)
        // {
        //     cout << "Robot " << agent_id << " has "
        //               << this->adg->getNumUnfinishedActions(agent_id)
        //               << " unfinished actions." << endl;
        // }
        // cout << "#####################" << endl;
    }

    if (invoke) {
        this->prev_invoke_planner_tick = sim_step;
        this->planner_running = true;
        this->planner_invoke_ticks.push_back(sim_step);
        if (this->screen > 0) {
            spdlog::info("Invoke planner at sim step: {}", sim_step);
            if (invoke_by >= 0) {
                spdlog::info("Invoked by robot {} with {} unfinished actions.",
                             invoke_by,
                             this->adg->getNumUnfinishedActions(invoke_by));
            }
        }
    }
    return invoke;
}

void ExecutionManager::recordStatsPerTick() {
    if (not this->adg->initialized) {
        return;
    }

    // Record ADG stats per tick
    this->adg->recordStatsPerTick();
}

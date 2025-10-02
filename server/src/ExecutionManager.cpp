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
      tick_per_robot(vector<int>(vm["num_robots"].as<int>(), 0)) {
    spdlog::info("Invoke policy: {}", planner_invoke_policy);
    spdlog::info("Look ahead tick: {}", look_ahead_tick);
    spdlog::info("Look ahead dist: {}", adg->getLookAheadDist());
    spdlog::info("Simulation window tick: {}", sim_window_tick);
}

void ExecutionManager::setupBackupPlanner() {
    spdlog::info("Setting up backup planner...");
    // Setup graph. We are computing heuristics here and in the planner, which
    // is redundant. Consider optimizing this later.
    string grid_type = this->_vm["grid_type"].as<std::string>();
    this->G.screen = this->screen;
    this->G.hold_endpoints = false;
    this->G.useDummyPaths = false;
    this->G._save_heuristics_table =
        this->_vm["save_heuristics_table"].as<bool>();
    this->G.rotation_time = this->_vm["rotation_time"].as<int>();
    // Grid type
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
    this->G.preprocessing(this->_vm["rotation"].as<bool>(), "map");

    // Setup backup single agent planner
    string single_solver_name =
        this->_vm["backup_single_agent_solver"].as<string>();
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

    // Setup backup MAPF planner
    string solver_name = this->_vm["backup_planner"].as<string>();
    spdlog::info("Backup planner: {}", solver_name);
    if (solver_name == "PIBT") {
        this->backup_planner = make_shared<PIBT>(this->G, *this->path_planner);
    } else {
        spdlog::error("Backup solver {} does not exist!", solver_name);
        exit(-1);
    }

    // Set parameters for the MAPF solver
    this->backup_planner->seed = this->seed;
    this->backup_planner->gen = mt19937(this->backup_planner->seed);
    this->backup_planner->screen = this->screen;
    this->backup_planner->simulation_window =
        this->_vm["sim_window_timestep"].as<int>();
    this->backup_planner->window = this->_vm["plan_window_timestep"].as<int>();
    this->backup_planner->hold_endpoints = false;
    this->backup_planner->num_of_agents = this->numRobots;
    this->backup_planner->k_robust = 0;

    // Set initialized flag
    this->backup_planner->set_initialized(true);
}

void ExecutionManager::saveStats() {
    if (!save_stats) {
        return;
    }

    // ifstream infile(output_filename);
    // bool exist = infile.good();
    // infile.close();
    // if (!exist) {
    //     ofstream addHeads(output_filename);
    //     addHeads << "Num of agent,Num of Finished Tasks" << endl;
    //     addHeads.close();
    // }
    // ofstream stats(output_filename, ios::app);
    // stats << numRobots << "," << this->adg->getNumFinishedTasks() << endl;
    // stats.close();

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
    ofstream stats(output_filename);
    stats << result.dump(4);  // Pretty print with 4 spaces

    // cout << "Statistics written to " << output_filename << endl;
    spdlog::info("Statistics written to {}", output_filename);
}

int ExecutionManager::getCurrSimStep() {
    // Return the minimum tick count among all robots
    return *min_element(tick_per_robot.begin(), tick_per_robot.end());
}

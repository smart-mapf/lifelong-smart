#include "ExecutionManager.h"

shared_ptr<ExecutionManager> server_ptr = nullptr;
std::mutex globalMutex;

void freezeSimulationIfNecessary(string RobotID) {
    lock_guard<mutex> guard(globalMutex);

    int robot_id = server_ptr->adg->startIndexToRobotID[RobotID];
    // Logic 1: freeze the simulation if at least one robot has no actions
    // if (server_ptr->adg->getNumUnfinishedActions(robot_id) <= 0) {
    //     server_ptr->freeze_simulation = true;
    //     if (server_ptr->screen > 0) {
    //         cout << "Robot " << robot_id
    //                   << " requests to freeze the simulation!" << endl;
    //     }
    // }

    // Logic 2: freeze the simulation if simulation tick has passed
    // more than `look_ahead_tick` than `prev_invoke_planner_tick`, or when
    // planner has never been invoked. Basically, if the planner does not
    // return in time. This is aim at synchronizing the simulation time with
    // world clock time
    int sim_step = server_ptr->getCurrSimStep();
    // int sim_step = server_ptr->time_step_tick;
    // spdlog::info("Checking freeze simulation at sim step {}, "
    //              "prev_invoke_planner_tick: {}, sim_window_tick: {}",
    //              sim_step, server_ptr->prev_invoke_planner_tick,
    //              server_ptr->sim_window_tick);
    if (sim_step == 0 && server_ptr->plannerNeverInvoked()) {
        server_ptr->freeze_simulation = true;
        if (server_ptr->screen > 0) {
            spdlog::info(
                "Robot {} requests to freeze the simulation at the first tick!",
                robot_id);
        }
    } else if (server_ptr->simTickElapsedFromLastInvoke(
                   server_ptr->look_ahead_tick) &&
               server_ptr->planner_running &&
               !server_ptr->simulationFinished()) {
        server_ptr->freeze_simulation = true;
        if (server_ptr->screen > 0) {
            spdlog::info(
                "Robot {} requests to freeze the simulation at sim step {} "
                "due to planner not return in time (synchronize)!",
                robot_id, sim_step);
        }
    }
}

// Return the simulation freeze status
bool isSimulationFrozen() {
    lock_guard<mutex> guard(globalMutex);
    return server_ptr->freeze_simulation;
}

string getRobotsLocation() {
    lock_guard<mutex> guard(globalMutex);

    // Return message as a JSON string
    json result_message = {};

    if (server_ptr->screen > 0) {
        spdlog::info("Get robot location query received!");
    }

    server_ptr->curr_robot_states = server_ptr->adg->computeCommitCut();

    vector<tuple<double, double, int>> robots_location;
    if (server_ptr->curr_robot_states.empty()) {
        result_message["robots_location"] = {};
        result_message["new_finished_tasks"] = {};
        result_message["initialized"] = false;
        return result_message.dump();
    }
    assert(static_cast<int>(server_ptr->curr_robot_states.size()) ==
           server_ptr->numRobots);

    for (auto& robot : server_ptr->curr_robot_states) {
        if (server_ptr->flipped_coord) {
            robots_location.emplace_back(make_tuple(
                robot.position.second, robot.position.first, robot.orient));
        } else {
            robots_location.emplace_back(make_tuple(
                robot.position.first, robot.position.second, robot.orient));
        }
    }

    set<int> new_finished_tasks = server_ptr->adg->updateFinishedTasks();

    server_ptr->tasks_finished_per_sec.push_back(make_tuple(
        new_finished_tasks.size(),
        server_ptr->getCurrSimStep() / server_ptr->ticks_per_second));

    server_ptr->robots_location = robots_location;

    result_message["robots_location"] = robots_location;
    result_message["new_finished_tasks"] = new_finished_tasks;
    result_message["initialized"] = true;
    return result_message.dump();
}

// Add a new MAPF plan to the ADG. Use backup planner if necessary
// new_plan: a vector of paths, each path is a vector of (row, col, t, task_id)
// Raw plan --> points --> Steps --> Actions
void addNewPlan(string& new_plan_json_str) {
    lock_guard<mutex> guard(globalMutex);

    json new_plan_json = json::parse(new_plan_json_str);

    vector<vector<tuple<int, int, double, int>>> new_plan;
    bool congested = false;
    bool success = false;

    if (new_plan_json.contains("success"))
        success = new_plan_json["success"].get<bool>();
    else {
        spdlog::warn("No success field in the new plan json! Assume failure.");
    }

    // Planner successfully returns.
    if (success) {
        new_plan = new_plan_json["plan"]
                       .get<vector<vector<tuple<int, int, double, int>>>>();
        congested = new_plan_json["congested"].get<bool>();
    }
    // Planner fails. Use the backup planner.
    else {
        if (!new_plan_json.contains("mapf_instance")) {
            spdlog::error("No MAPF instance provided when planner fails!");
            exit(-1);
        }

        json mapf_instance = new_plan_json["mapf_instance"];

        if (!mapf_instance.contains("starts") ||
            !mapf_instance.contains("goals")) {
            spdlog::error(
                "MAPF instance does not contain starts or goals when planner "
                "fails!");
            exit(-1);
        }

        auto starts = mapf_instance.at("starts").get<vector<State>>();
        auto goal_locations =
            mapf_instance.at("goals").get<vector<vector<Task>>>();

        // Plan using the backup planner
        spdlog::info("Planner fails, using backup planner");
        server_ptr->backup_planner->clear();
        server_ptr->backup_planner->run(starts, goal_locations);
        new_plan =
            server_ptr->backup_planner->convert_path_to_smart(goal_locations);
        congested = server_ptr->backup_planner->congested();
    }

    // TODO: consider backup planner for the following entries.
    // Store stats, if available
    if (new_plan_json.contains("stats")) {
        server_ptr->planner_stats = new_plan_json["stats"];
    }

    // update backup tasks, if available
    if (new_plan_json.contains("backup_tasks")) {
        auto backup_tasks = new_plan_json["backup_tasks"].get<set<int>>();
        server_ptr->adg->backup_tasks = backup_tasks;
    }

    if (congested) {
        // Stop the server early.
        // NOTE: We cannot call closeServer directly because we need to ensure
        // the clients (robots) are closed. So we set a flag to let the robots
        // know the simulation should be stopped and the robots will call
        // closeServer.
        // cout << "Congested simulation detected, stopping the
        // simulation!"
        //           << endl;
        spdlog::info("Congested simulation detected, stopping the simulation!");
        server_ptr->congested_sim = true;
    }

    // Convert raw plan to points
    vector<vector<Step>> steps;
    assert(new_plan.size() == server_ptr->numRobots);
    for (int agent_id = 0; agent_id < server_ptr->numRobots; agent_id++) {
        vector<Point> points;
        vector<Step> curr_steps;
        for (auto& step : new_plan[agent_id]) {
            points.emplace_back(
                Point(get<0>(step), get<1>(step), get<2>(step), get<3>(step)));
        }

        // Convert points to steps, which adds rotational states to the plan
        // returned by the MAPF planner if needed
        server_ptr->parser.AgentPathToSteps(
            points, curr_steps, server_ptr->curr_robot_states[agent_id].orient,
            agent_id);
        steps.push_back(curr_steps);
    }

    // Convert steps to actions, each two consecutive steps will be transformed
    // to at most two actions.
    vector<vector<Action>> actions;
    actions =
        server_ptr->parser.StepsToActions(steps, server_ptr->flipped_coord);
#ifdef DEBUG
    cout << "Finish action process, plan size: " << actions.size() << endl;
#endif

    server_ptr->adg->addMAPFPlan(actions);

    // Defreeze the simulation if it was frozen
    if (server_ptr->freeze_simulation) {
        server_ptr->freeze_simulation = false;
        if (server_ptr->screen > 0) {
            // cout << "Simulation is de-frozen after adding a new plan!"
            //           << endl;
            spdlog::info("Simulation is de-frozen after adding a new plan!");
        }
    }

    // Planner stop running
    server_ptr->planner_running = false;

#ifdef DEBUG
    cout << "Finish add plan" << endl;
#endif
}

string actionFinished(string& robot_id_str, int node_ID) {
    lock_guard<mutex> guard(globalMutex);
    if (not server_ptr->adg->initialized) {
        cerr << "ExecutionManager::ExecutionManager: server_ptr is not "
                "initialized"
             << endl;
        exit(-1);
        return "None";
    }
    int agent_id = server_ptr->adg->startIndexToRobotID[robot_id_str];
    bool status_update = server_ptr->adg->updateFinishedNode(agent_id, node_ID);

    return "None";
}

void init(string RobotID, tuple<int, int> init_loc) {
    lock_guard<mutex> guard(globalMutex);
    // Initialize the backup planner if it has not been initialized
    if (server_ptr->backup_planner == nullptr ||
        !server_ptr->backup_planner->is_initialized()) {
        server_ptr->setupBackupPlanner();
    }

    // Inform ADG with the current robot's initial location
    server_ptr->adg->createRobotIDToStartIndexMaps(RobotID, init_loc);
}

bool isInitialized() {
    lock_guard<mutex> guard(globalMutex);
    return server_ptr->adg->initialized;
}

inline void insertNewGoal(int agent_id, pair<int, int> tmp_loc,
                          vector<vector<tuple<int, int, double>>>& new_goals,
                          double goal_orient = -1) {
    int tmp_x = tmp_loc.first;
    int tmp_y = tmp_loc.second;
    new_goals[agent_id].emplace_back(tmp_x, tmp_y, goal_orient);
}

// void updateStats(double total_wait_time, int capacity) {
//     if (server_ptr->screen > 0) {
//         cout << "update the stats with: " << total_wait_time
//                   << ", and capacity: " << capacity << endl;
//     }

//     server_ptr->total_wait_time = total_wait_time;
//     server_ptr->transport_capacity = capacity;
// }

void closeServer(rpc::server& srv) {
    spdlog::info("Closing server at port {}", server_ptr->port);
    // lock_guard<mutex> guard(globalMutex);
    // cout <<
    // "############################################################"
    //           << endl;
    // cout << "Closing server at port " << server_ptr->port << endl;
    // cout << "Sim count " << server_ptr->tick_per_robot[0] << endl;
    // cout << "Num of finished tasks: "
    //           << server_ptr->adg->getNumFinishedTasks() << endl;

    auto end = std::chrono::steady_clock::now();
    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(
        end - server_ptr->start_time);
    server_ptr->overall_runtime = elapsed_seconds.count();
    spdlog::info("Simulation count: {}", server_ptr->tick_per_robot[0]);
    // spdlog::info("Number of actual finished tasks: {}",
    //              server_ptr->adg->countFinishedTasks());
    // spdlog::info("Average num of rotation per robot: {:.2f}",
    //              server_ptr->adg->avgRotation());
    spdlog::info("Number of finished tasks: {}",
                 server_ptr->adg->getNumFinishedTasks());
    spdlog::info("Number of finished backup tasks: {}",
                 server_ptr->adg->getNumFinishedBackupTasks());
    spdlog::info("Overall runtime: {:.2f} seconds",
                 server_ptr->overall_runtime);

    server_ptr->saveStats();
    // cout <<
    // "############################################################"
    //           << endl;
    srv.close_sessions();
    srv.stop();
    spdlog::info("Server closed successfully.");
}

SIM_PLAN update(string RobotID) {
    lock_guard<mutex> guard(globalMutex);
    if (not server_ptr->adg->initialized or
        not server_ptr->adg->get_initial_plan) {
        return {};
    }
    // printf("robot ID: %s\n", RobotID.c_str());

    int Robot_ID = server_ptr->adg->startIndexToRobotID[RobotID];
    // printf("robot id is: %d\n", Robot_ID);
    // if (server_ptr->step_cnt % 20 == 0 and Robot_ID == 0) {
    //     server_ptr->adg->printProgress();
    // }
    // server_ptr->step_cnt++;
    return server_ptr->adg->getPlan(Robot_ID);
}

// void updateSimFinishTime(string& robot_id_str, int sim_step) {
//     int robot_id = server_ptr->adg->startIndexToRobotID[robot_id_str];
//     if (server_ptr->agent_finish_sim_step[robot_id] < 0) {
//         server_ptr->agent_finish_sim_step[robot_id] = sim_step;
//         server_ptr->latest_arr_sim_step = sim_step;
//     }
// }

bool simStatus() {
    lock_guard<mutex> guard(globalMutex);
    // Check if the simulation is congested
    if (server_ptr->congested_sim) {
        return true;  // If the simulation is congested, we stop the simulation
    }

    int sim_step = server_ptr->getCurrSimStep();
    return sim_step >= server_ptr->total_sim_step_tick;
}

// Return end_sim: true if all robots have finished their simulation steps
bool updateSimStep(string RobotID) {
    lock_guard<mutex> guard(globalMutex);

    if (server_ptr->congested_sim) {
        return true;  // If the simulation is congested, we stop the simulation
    }

    int Robot_ID = server_ptr->adg->startIndexToRobotID[RobotID];
    server_ptr->tick_per_robot[Robot_ID]++;

    bool end_sim = true;
    for (auto tick : server_ptr->tick_per_robot) {
        if (tick < server_ptr->total_sim_step_tick) {
            end_sim = false;
            break;
        }
    }
    return end_sim;
}

// Update the simulation step tick and return whether the simulation should end
// (i.e., all robots have finished their simulation steps).
// This function is called by the client to update the simulation step tick.
// We stop the simulation if it is congested or if the updated tick is greater
// than or equal to the total simulation step tick.
// tuple<int, bool> updateSimStep() {
//     lock_guard<mutex> guard(globalMutex);
//     server_ptr->time_step_tick++;

//     bool end_sim =
//         server_ptr->time_step_tick >= server_ptr->total_sim_step_tick ||
//         server_ptr->congested_sim;
//     return make_tuple(server_ptr->time_step_tick, end_sim);
// }

bool invokePlanner() {
    lock_guard<mutex> guard(globalMutex);

    // Should take the min of the ticks of all the robots
    int sim_step = server_ptr->getCurrSimStep();
    bool invoke = false;
    int invoke_by = -1;

    // Default: invoke planner every sim_window_tick
    if (server_ptr->planner_invoke_policy == "default") {
        invoke = (sim_step == 0 || server_ptr->simTickElapsedFromLastInvoke(
                                       server_ptr->sim_window_tick)) &&
                 !server_ptr->simulationFinished() &&
                 sim_step != server_ptr->prev_invoke_planner_tick;
    }
    // No action: invoke planner when the number of actions left for a robot
    // is less than look_ahead_dist, and at least 1 has passed
    // since last invocation.
    else if (server_ptr->planner_invoke_policy == "no_action") {
        for (int agent_id = 0; agent_id < server_ptr->numRobots; agent_id++) {
            if (server_ptr->adg->getNumUnfinishedActions(agent_id) <=
                server_ptr->adg->getLookAheadDist()) {
                invoke = true;
                invoke_by = agent_id;
                break;
            }
        }

        // Ensure we have not reached the total simulation step tick
        invoke &= !server_ptr->simulationFinished();

        // Ensure at least 1 has passed since last invocation
        if (invoke && !server_ptr->plannerNeverInvoked() &&
            !server_ptr->simTickElapsedFromLastInvoke(1)) {
            if (server_ptr->screen > 1) {
                spdlog::info(
                    "Timestep {}: Attempt to invoke by {}, but skipped to "
                    "ensure {} has passed since last invocation at {}.",
                    sim_step, invoke_by, 1,
                    server_ptr->prev_invoke_planner_tick);
            }
            invoke = false;
            invoke_by = -1;
        }
    } else {
        spdlog::error("Unknown planner invoke policy: {}",
                      server_ptr->planner_invoke_policy);
        exit(-1);
    }

    // First invoke, start record runtime
    if (invoke && sim_step == 0) {
        server_ptr->start_time = std::chrono::steady_clock::now();
        spdlog::info("Start time recorded at sim step 0.");
    }

    // Print the unfinished actions for each robot
    if (server_ptr->screen > 0) {
        // cout << "#####################" << endl;
        // cout << "Checking if planner should be invoked at sim step: "
        //      << sim_step << ", invoke: " << invoke << endl;
        // spdlog::info("Checking if planner should be invoked at sim step: {},
        // "
        //              "invoke: {}",
        //              sim_step, invoke);
        // cout << "Unfinished actions for each robot at sim step "
        //           << sim_step << ":" << endl;
        // for (int agent_id = 0; agent_id < server_ptr->numRobots; agent_id++)
        // {
        //     cout << "Robot " << agent_id << " has "
        //               << server_ptr->adg->getNumUnfinishedActions(agent_id)
        //               << " unfinished actions." << endl;
        // }
        // cout << "#####################" << endl;
    }

    if (invoke) {
        server_ptr->prev_invoke_planner_tick = sim_step;
        server_ptr->planner_running = true;
        server_ptr->planner_invoke_ticks.push_back(sim_step);
        if (server_ptr->screen > 0) {
            spdlog::info("Invoke planner at sim step: {}", sim_step);
            if (invoke_by >= 0) {
                spdlog::info(
                    "Invoked by robot {} with {} unfinished actions.",
                    invoke_by,
                    server_ptr->adg->getNumUnfinishedActions(invoke_by));
            }
        }
    }
    return invoke;
}

int getNumUnfinishedActions(string RobotID) {
    lock_guard<mutex> guard(globalMutex);
    int Robot_ID = server_ptr->adg->startIndexToRobotID[RobotID];
    return server_ptr->adg->getNumUnfinishedActions(Robot_ID);
}

void recordStatsPerTick() {
    lock_guard<mutex> guard(globalMutex);
    if (not server_ptr->adg->initialized) {
        return;
    }

    // Record ADG stats per tick
    server_ptr->adg->recordStatsPerTick();
}

int main(int argc, char** argv) {
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
            ("help", "produce help message")
            // params for the input instance and experiment settings
            ("num_robots,k", po::value<int>()->required(), "number of robots in server")
            ("port_number,n", po::value<int>()->default_value(8080), "rpc port number")
            ("output_file,o", po::value<string>()->default_value("stats.json"), "output statistic filename")
            ("save_stats,s", po::value<bool>()->default_value(false), "write to files some detailed statistics")
            ("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
            ("planner_invoke_policy", po::value<string>()->default_value("default"), "planner invoke policy: default or no_action")
            ("sim_window_tick,w", po::value<int>()->default_value(50), "invoke planner every sim_window_tick (default: 50)")
            ("sim_window_timestep", po::value<int>()->default_value(5), "invoke planner every sim_window_timestep (default: 5)")
            ("plan_window_timestep", po::value<int>()->default_value(5), "plan for this many timesteps (default: 5)")
            ("total_sim_step_tick,t", po::value<int>()->default_value(1200), "total simulation step tick (default: 1)")
            ("ticks_per_second,f", po::value<int>()->default_value(10), "ticks per second for the simulation (default: 10)")
            ("look_ahead_dist,l", po::value<int>()->default_value(5), "look ahead # of actions for the robot to query its location")
            ("look_ahead_tick,m", po::value<int>()->default_value(5), "look ahead tick for the robot to query its location")
            ("seed", po::value<int>()->default_value(0), "random seed for the simulation (default: 0)")
            ("backup_planner", po::value<string>()->default_value("PIBT"), "backup planner: PIBT or none")
            ("backup_single_agent_solver", po::value<string>()->default_value("ASTAR"), "backup single-agent solver: ASTAR, SIPP")
            ("map", po::value<string>()->default_value("maps/empty_32_32.json"), "map filename")
            ("grid_type", po::value<string>()->default_value("four_connected"), "grid type: four_connected, four_connected_with_diagonal, eight_connected")
            ("rotation", po::value<bool>()->default_value(false), "consider rotation when planning and executing")
            ("rotation_time", po::value<int>()->default_value(1), "rotation time for the robots (default: 1)")
            ("save_heuristics_table", po::value<bool>()->default_value(false), "save heuristics table to speed up single-agent pathfinding")
            ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return 1;
    }
    po::notify(vm);
    string filename = "none";
    int port_number = vm["port_number"].as<int>();

    int seed = vm["seed"].as<int>();
    srand(seed);

    server_ptr = make_shared<ExecutionManager>(vm);

    // Set up logger
    auto console_logger = spdlog::default_logger()->clone("ExecutionManager");
    spdlog::set_default_logger(console_logger);

    // Setup the server to listen on the specified port number
    rpc::server srv(port_number);

    // Bind the function to the server
    // TODO: Remove unused functions
    srv.bind("receive_update", &actionFinished);
    srv.bind("init", &init);
    srv.bind("is_initialized", &isInitialized);
    srv.bind("get_location", &getRobotsLocation);
    srv.bind("add_plan", &addNewPlan);
    srv.bind("update", &update);
    // srv.bind("update_finish_agent", &updateSimFinishTime);
    // srv.bind("update_stats", &updateStats);
    srv.bind("update_sim_step", &updateSimStep);
    srv.bind("invoke_planner", &invokePlanner);
    srv.bind("close_server", [&srv]() { closeServer(srv); });
    srv.bind("get_num_unfinished_actions", &getNumUnfinishedActions);
    srv.bind("freeze_simulation_if_necessary", &freezeSimulationIfNecessary);
    srv.bind("is_simulation_frozen", &isSimulationFrozen);
    srv.bind("sim_status", &simStatus);
    srv.bind("record_stats_per_tick", &recordStatsPerTick);
    srv.run();  // Start the server, blocking call

    return 0;
}
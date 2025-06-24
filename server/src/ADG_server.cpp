#include "ADG_server.h"

// std::vector<std::chrono::steady_clock::time_point>
//     startTimers;  // Start times for each robot
std::shared_ptr<ADG_Server> server_ptr = nullptr;
// =======================

ADG_Server::ADG_Server(int num_robots, std::string target_output_filename,
                       bool save_stats, int screen, int port,
                       int total_sim_step_tick, int ticks_per_second,
                       int look_ahead_dist, int seed)
    : output_filename(target_output_filename),
      save_stats(save_stats),
      screen(screen),
      port(port),
      total_sim_step_tick(total_sim_step_tick),
      ticks_per_second(ticks_per_second),
      seed(seed) {
    adg = std::make_shared<ADG>(num_robots, screen, look_ahead_dist);

    numRobots = adg->numRobots();
    // agent_finish_time.resize(numRobots, -1);
    // agents_finish.resize(numRobots, false);
    // agent_finish_sim_step.resize(numRobots, -1);
    tick_per_robot.resize(numRobots, 0);

    // startTimers.resize(numRobots);
}

void ADG_Server::saveStats() {
    if (!save_stats) {
        return;
    }

    // std::ifstream infile(output_filename);
    // bool exist = infile.good();
    // infile.close();
    // if (!exist) {
    //     ofstream addHeads(output_filename);
    //     addHeads << "Num of agent,Num of Finished Tasks" << endl;
    //     addHeads.close();
    // }
    // ofstream stats(output_filename, std::ios::app);
    // stats << numRobots << "," << this->adg->getNumFinishedTasks() << endl;
    // stats.close();


    // Compute throughput as the number of finished tasks per sim second
    int total_finished_tasks = adg->getNumFinishedTasks();
    int sim_seconds = total_sim_step_tick / ticks_per_second;
    double throughput = static_cast<double>(total_finished_tasks) / sim_seconds;
    json result = {
        {"total_finished_tasks", total_finished_tasks},
        {"throughput", throughput},
        {"success", true}
    };

    // Write the statistics to the output file
    std::ofstream stats(output_filename);
    stats << result.dump(4);  // Pretty print with 4 spaces


    std::cout << "Statistics written to " << output_filename << std::endl;
}

// Each robot can requests to freeze the simulation if it does not have enough
// actions
void freezeSimulationIfNecessary(std::string RobotID) {
    std::lock_guard<std::mutex> guard(globalMutex);

    int robot_id = server_ptr->adg->startIndexToRobotID[RobotID];
    if (server_ptr->adg->getNumUnfinishedActions(robot_id) <= 0) {
        server_ptr->freeze_simulation = true;
        if (server_ptr->screen > 0) {
            std::cout << "Robot " << robot_id
                      << " requests to freeze the simulation!" << std::endl;
        }
    }
}

// Return the simulation freeze status
bool isSimulationFrozen() {
    std::lock_guard<std::mutex> guard(globalMutex);
    return server_ptr->freeze_simulation;
}

string getRobotsLocation() {
    // Return message as a JSON string
    json result_message = {};

    std::lock_guard<std::mutex> guard(globalMutex);
    if (server_ptr->screen > 0) {
        std::cout << "Get robot location query received!" << std::endl;
    }

    server_ptr->curr_robot_states = server_ptr->adg->computeCommitCut();

    std::vector<std::pair<double, double>> robots_location;
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
            robots_location.emplace_back(robot.position.second,
                                         robot.position.first);
        } else {
            robots_location.emplace_back(robot.position.first,
                                         robot.position.second);
        }
    }

    set<int> new_finished_tasks = server_ptr->adg->updateFinishedTasks();

    server_ptr->robots_location = robots_location;

    result_message["robots_location"] = robots_location;
    result_message["new_finished_tasks"] = new_finished_tasks;
    result_message["initialized"] = true;
    return result_message.dump();
}

void addNewPlan(
    std::vector<std::vector<std::tuple<int, int, double, int>>>& new_plan) {
    // x, y and time
    std::lock_guard<std::mutex> guard(globalMutex);
    std::vector<std::vector<Step>> steps;
    assert(new_plan.size() == server_ptr->numRobots);
    for (int agent_id = 0; agent_id < server_ptr->numRobots; agent_id++) {
        std::vector<Point> points;
        std::vector<Step> curr_steps;
        for (auto& step : new_plan[agent_id]) {
            points.emplace_back(Point(std::get<0>(step), std::get<1>(step),
                                      std::get<2>(step), std::get<3>(step)));
        }

        // Convert points to steps, which adds rotational states to the plan
        // returned by the MAPF planner if needed
        AgentPathToSteps(points, curr_steps,
                         server_ptr->curr_robot_states[agent_id].orient,
                         agent_id);
        steps.push_back(curr_steps);
    }

    // Convert steps to actions, each two consecutive steps will be transformed
    // to at most two actions.
    std::vector<std::vector<Action>> actions;
    actions = StepsToActions(steps, server_ptr->flipped_coord);
#ifdef DEBUG
    std::cout << "Finish action process, plan size: " << actions.size()
              << std::endl;
#endif

    server_ptr->adg->addMAPFPlan(actions);

    // Defreeze the simulation if it was frozen
    if (server_ptr->freeze_simulation) {
        server_ptr->freeze_simulation = false;
        if (server_ptr->screen > 0) {
            std::cout << "Simulation is de-frozen after adding a new plan!"
                      << std::endl;
        }
    }

#ifdef DEBUG
    std::cout << "Finish add plan" << std::endl;
#endif
}

std::string actionFinished(std::string& robot_id_str, int node_ID) {
    std::lock_guard<std::mutex> guard(globalMutex);
    if (not server_ptr->adg->initialized) {
        std::cerr << "ADG_Server::ADG_Server: server_ptr is not initialized"
                  << std::endl;
        exit(-1);
        return "None";
    }
    int agent_id = server_ptr->adg->startIndexToRobotID[robot_id_str];
    bool status_update = server_ptr->adg->updateFinishedNode(agent_id, node_ID);

    return "None";
}

void init(std::string RobotID) {
    std::lock_guard<std::mutex> guard(globalMutex);
    server_ptr->adg->createRobotIDToStartIndexMaps(RobotID);
}

bool isInitialized() {
    std::lock_guard<std::mutex> guard(globalMutex);
    return server_ptr->adg->initialized;
}

inline void insertNewGoal(
    int agent_id, std::pair<int, int> tmp_loc,
    std::vector<std::vector<std::tuple<int, int, double>>>& new_goals,
    double goal_orient = -1) {
    int tmp_x = tmp_loc.first;
    int tmp_y = tmp_loc.second;
    new_goals[agent_id].emplace_back(tmp_x, tmp_y, goal_orient);
}

// void updateStats(double total_wait_time, int capacity) {
//     if (server_ptr->screen > 0) {
//         std::cout << "update the stats with: " << total_wait_time
//                   << ", and capacity: " << capacity << std::endl;
//     }

//     server_ptr->total_wait_time = total_wait_time;
//     server_ptr->transport_capacity = capacity;
// }

void closeServer(rpc::server& srv) {
    std::lock_guard<std::mutex> guard(globalMutex);
    std::cout << "############################################################"
              << std::endl;
    std::cout << "Closing server at port " << server_ptr->port << std::endl;
    std::cout << "Sim count " << server_ptr->tick_per_robot[0] << std::endl;
    std::cout << "Num of finished tasks: "
              << server_ptr->adg->getNumFinishedTasks() << std::endl;
    server_ptr->saveStats();
    std::cout << "############################################################"
              << std::endl;
    srv.close_sessions();
    srv.stop();
}

std::vector<
    std::tuple<std::string, int, double, std::string, std::pair<double, double>,
               std::pair<double, double>, int>>
update(std::string RobotID) {
    std::lock_guard<std::mutex> guard(globalMutex);
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

// void updateSimFinishTime(std::string& robot_id_str, int sim_step) {
//     int robot_id = server_ptr->adg->startIndexToRobotID[robot_id_str];
//     if (server_ptr->agent_finish_sim_step[robot_id] < 0) {
//         server_ptr->agent_finish_sim_step[robot_id] = sim_step;
//         server_ptr->latest_arr_sim_step = sim_step;
//     }
// }

// Return end_sim: true if all robots have finished their simulation steps
bool updateSimStep(std::string RobotID) {
    std::lock_guard<std::mutex> guard(globalMutex);
    int Robot_ID = server_ptr->adg->startIndexToRobotID[RobotID];
    server_ptr->tick_per_robot[Robot_ID]++;

    bool end_sim = true;
    for (auto tick : server_ptr->tick_per_robot) {
        if (tick < server_ptr->total_sim_step_tick) {
            end_sim = false;
            break;
        }
    }
    bool end_robot =
        server_ptr->tick_per_robot[Robot_ID] >= server_ptr->total_sim_step_tick;
    return end_sim;
}

// Invoke planner when the number of actions left for a robot is less than a
// threshold.
bool invokePlanner() {
    std::lock_guard<std::mutex> guard(globalMutex);
    int sim_step = *std::min_element(server_ptr->tick_per_robot.begin(),
                                     server_ptr->tick_per_robot.end());
    // ########## OLD logic: invoke planner every sim_window_tick ##########
    // Should take the min of the ticks of all the robots
    // int sim_step = *std::min_element(server_ptr->tick_per_robot.begin(),
    //                                  server_ptr->tick_per_robot.end());

    // // Invoke planner every sim_window_tick
    // bool invoke = sim_step % server_ptr->sim_window_tick == 0 &&
    //               sim_step < server_ptr->total_sim_step_tick;
    // ########## END OLD logic ##########

    // Invoke planner if the number of actions left for a robot is less than
    // look_ahead_dist
    bool invoke = false;
    int invoke_by = -1;
    for (int agent_id = 0; agent_id < server_ptr->numRobots; agent_id++) {
        if (server_ptr->adg->getNumUnfinishedActions(agent_id) <=
            server_ptr->adg->getLookAheadDist()) {
            invoke = true;
            invoke_by = agent_id;
            break;
        }
    }

    // Print the unfinished actions for each robot
    if (server_ptr->screen > 0) {
        // cout << "#####################" << std::endl;
        // cout << "Checking if planner should be invoked at sim step: "
        //      << sim_step << ", invoke: " << invoke << std::endl;
        // std::cout << "Unfinished actions for each robot at sim step "
        //           << sim_step << ":" << std::endl;
        // for (int agent_id = 0; agent_id < server_ptr->numRobots; agent_id++)
        // {
        //     std::cout << "Robot " << agent_id << " has "
        //               << server_ptr->adg->getNumUnfinishedActions(agent_id)
        //               << " unfinished actions." << std::endl;
        // }
        // cout << "#####################" << std::endl;
    }

    if (invoke && server_ptr->screen > 0) {
        std::cout << "Invoke planner at sim step: " << sim_step << std::endl;
        for (int agent_id = 0; agent_id < server_ptr->numRobots; agent_id++) {
            // std::cout << "Robot " << agent_id << " has "
            //           << server_ptr->adg->getNumUnfinishedActions(agent_id)
            //           << " unfinished actions." << std::endl;
        }
        if (invoke_by >= 0) {
            std::cout << "Invoke planner by robot " << invoke_by << " with "
                      << server_ptr->adg->getNumUnfinishedActions(invoke_by)
                      << " unfinished actions." << std::endl;
        }
    }
    return invoke;
}

int getNumUnfinishedActions(std::string RobotID) {
    std::lock_guard<std::mutex> guard(globalMutex);
    int Robot_ID = server_ptr->adg->startIndexToRobotID[RobotID];
    return server_ptr->adg->getNumUnfinishedActions(Robot_ID);
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
            ("total_sim_step_tick,t", po::value<int>()->default_value(1200), "total simulation step tick (default: 1)")
            ("ticks_per_second,f", po::value<int>()->default_value(10), "ticks per second for the simulation (default: 10)")
            ("look_ahead_dist,l", po::value<int>()->default_value(5), "look ahead # of actions for the robot to query its location")
            ("seed", po::value<int>()->default_value(0), "random seed for the simulation (default: 0)")
            ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return 1;
    }
    po::notify(vm);
    std::string filename = "none";
    int port_number = vm["port_number"].as<int>();

    int seed = vm["seed"].as<int>();
    srand(seed);

    server_ptr = std::make_shared<ADG_Server>(
        vm["num_robots"].as<int>(), vm["output_file"].as<std::string>(),
        vm["save_stats"].as<bool>(), vm["screen"].as<int>(), port_number,
        vm["total_sim_step_tick"].as<int>(), vm["ticks_per_second"].as<int>(),
        vm["look_ahead_dist"].as<int>(), seed);

    rpc::server srv(port_number);  // Setup the server to listen on the
    // specified port number

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
    srv.run();  // Start the server, blocking call

    return 0;
}
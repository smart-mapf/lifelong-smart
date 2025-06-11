#include "ADG_server.h"

std::vector<std::chrono::steady_clock::time_point>
    startTimers;  // Start times for each robot
std::shared_ptr<ADG_Server> server_ptr = nullptr;
// =======================

ADG_Server::ADG_Server(int num_robots, std::string target_output_filename,
                       bool save_stats, int screen, int port,
                       int total_sim_step_tick)
    : output_filename(target_output_filename),
      save_stats(save_stats),
      screen(screen),
      port(port),
      total_sim_step_tick(total_sim_step_tick) {
    adg = std::make_shared<ADG>(num_robots, screen);

    numRobots = adg->numRobots();
    agent_finish_time.resize(numRobots, -1);
    agents_finish.resize(numRobots, false);
    agent_finish_sim_step.resize(numRobots, -1);
    tick_per_robot.resize(numRobots, 0);

    startTimers.resize(numRobots);
}

void ADG_Server::saveStats(int selector_wait_t, int capacity) {
    if (!save_stats) {
        return;
    }

    std::ifstream infile(output_filename);
    bool exist = infile.good();
    infile.close();
    if (!exist) {
        ofstream addHeads(output_filename);
        addHeads << "Num of agent,Num of Finished Tasks" << endl;
        addHeads.close();
    }
    ofstream stats(output_filename, std::ios::app);
    stats << numRobots << "," << this->adg->getNumFinishedTasks() << endl;
    stats.close();
    std::cout << "Statistics written to " << output_filename << std::endl;
}

string getRobotsLocation(int look_ahead_dist) {
    // Return message as a JSON string
    json result_message = {};

    std::lock_guard<std::mutex> guard(globalMutex);
    if (server_ptr->screen > 0) {
        std::cout << "Get robot location query received! lookahead dist: "
                  << look_ahead_dist << std::endl;
    }

    server_ptr->curr_robot_states =
        server_ptr->adg->computeCommitCut(look_ahead_dist);

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

    // TODO@jingtian: set a task as finished when it is enqueued
    // std::vector< std::unordered_set<int> > task_status;
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
    std::vector<std::vector<Step>> raw_plan;
    assert(new_plan.size() == server_ptr->numRobots);
    for (int agent_id = 0; agent_id < server_ptr->numRobots; agent_id++) {
        std::vector<Point> points;
        std::vector<Step> tmp_plan;
        for (auto& step : new_plan[agent_id]) {
            points.emplace_back(Point(std::get<0>(step), std::get<1>(step),
                                      std::get<2>(step), std::get<3>(step)));
        }
        processAgentActions(points, tmp_plan,
                            server_ptr->curr_robot_states[agent_id].orient,
                            agent_id);
        raw_plan.push_back(tmp_plan);
    }

    std::vector<std::vector<Action>> plans;
    plans = processActions(raw_plan, server_ptr->flipped_coord);
#ifdef DEBUG
    std::cout << "Finish action process, plan size: " << plans.size()
              << std::endl;
#endif

    server_ptr->adg->addMAPFPlan(plans);

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

    // std::pair<double, double> curr_pos =
    // server_ptr->adg->getRobotPosition(agent_id); if
    // (server_ptr->adg->isTaskNode(agent_id, node_ID)) {
    //     std::cout << "receive confirmation for robot id: " << agent_id << ",
    //     at node id: "<< node_ID << "with current position at: " <<
    //     curr_pos.first << ", " << curr_pos.second << std::endl;
    //     server_ptr->mobile_manager->finishTask(agent_id,
    //     server_ptr->curr_mobile_tasks[agent_id].front());
    // }

    // printf("goal::(%f, %f), curr_pos::(%f, %f)\n",
    // server_ptr->adg->getActionGoal(Robot_ID, node_ID).first,
    //     server_ptr->adg->getActionGoal(Robot_ID, node_ID).second,
    //     curr_pos.first, curr_pos.second);
    // if (server_ptr->task_manager_ptr->isAgentFinished(Robot_ID, curr_pos)) {
    //     auto endTime = std::chrono::steady_clock::now();
    //     auto diff = endTime - startTimers[Robot_ID];
    //     double duration =
    //     std::chrono::duration_cast<std::chrono::duration<double>>(diff).count();
    //     printf("Agent %d reach its goal!\n", Robot_ID);
    //     // if (server_ptr->agent_finish_time[Robot_ID] < 0) {
    //     //     server_ptr->agent_finish_time[Robot_ID] = duration;
    //     //     server_ptr->agents_finish[Robot_ID] = true;
    //     // }
    // }
    return "None";
}

void init(std::string RobotID) {
    std::lock_guard<std::mutex> guard(globalMutex);
    server_ptr->adg->createRobotIDToStartIndexMaps(RobotID);
}

inline void insertNewGoal(
    int agent_id, std::pair<int, int> tmp_loc,
    std::vector<std::vector<std::tuple<int, int, double>>>& new_goals,
    double goal_orient = -1) {
    int tmp_x = tmp_loc.first;
    int tmp_y = tmp_loc.second;
    new_goals[agent_id].emplace_back(tmp_x, tmp_y, goal_orient);
}

void updateStats(double total_wait_time, int capacity) {
    if (server_ptr->screen > 0) {
        std::cout << "update the stats with: " << total_wait_time
                  << ", and capacity: " << capacity << std::endl;
    }

    server_ptr->total_wait_time = total_wait_time;
    server_ptr->transport_capacity = capacity;
}

void closeServer(rpc::server& srv) {
    std::lock_guard<std::mutex> guard(globalMutex);
    std::cout << "############################################################"
              << std::endl;
    std::cout << "Closing server at port " << server_ptr->port << std::endl;
    std::cout << "Sim count " << server_ptr->step_cnt << std::endl;
    std::cout << "Num of finished tasks: "
              << server_ptr->adg->getNumFinishedTasks() << std::endl;
    server_ptr->saveStats(server_ptr->total_wait_time,
                          server_ptr->transport_capacity);
    std::cout << "############################################################"
              << std::endl;
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

void updateSimFinishTime(std::string& robot_id_str, int sim_step) {
    int robot_id = server_ptr->adg->startIndexToRobotID[robot_id_str];
    if (server_ptr->agent_finish_sim_step[robot_id] < 0) {
        server_ptr->agent_finish_sim_step[robot_id] = sim_step;
        server_ptr->latest_arr_sim_step = sim_step;
    }
}

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

int main(int argc, char** argv) {
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
            ("help", "produce help message")
            // params for the input instance and experiment settings
            ("path_file,p", po::value<string>(), "input file for path")
            ("num_robots,k", po::value<int>()->required(), "number of robots in server")
            ("port_number,n", po::value<int>()->default_value(8080), "rpc port number")
            ("output_file,o", po::value<string>()->default_value("stats.csv"), "output statistic filename")
            ("save_stats,s", po::value<bool>()->default_value(false), "write to files some detailed statistics")
            ("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
            ("total_sim_step_tick,t", po::value<int>()->default_value(1200), "total simulation step tick (default: 1)")
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

    server_ptr = std::make_shared<ADG_Server>(
        vm["num_robots"].as<int>(), vm["output_file"].as<std::string>(),
        vm["save_stats"].as<bool>(), vm["screen"].as<int>(), port_number,
        vm["total_sim_step_tick"].as<int>());

    rpc::server srv(port_number);  // Setup the server to listen on the
    // specified port number

    // Bind the function to the server
    // TODO: Remove unused functions
    srv.bind("receive_update", &actionFinished);
    srv.bind("init", &init);
    srv.bind("get_location", &getRobotsLocation);
    srv.bind("add_plan", &addNewPlan);
    srv.bind("update", &update);
    srv.bind("update_finish_agent", &updateSimFinishTime);

    srv.bind("update_stats", &updateStats);
    srv.bind("update_sim_step", &updateSimStep);
    srv.bind("close_server", [&srv]() { closeServer(srv); });
    srv.run();  // Start the server, blocking call

    return 0;
}
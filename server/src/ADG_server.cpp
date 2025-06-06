#include "ADG_server.h"

std::vector<std::chrono::steady_clock::time_point>
    startTimers;  // Start times for each robot
std::shared_ptr<ADG_Server> server_ptr = nullptr;
// =======================

ADG_Server::ADG_Server(int num_robots, int num_pickers,
                       std::string& target_output_filename,
                       std::string map_name, std::string scen_name,
                       std::string method_name)
    : curr_map_name(map_name),
      curr_scen_name(scen_name),
      curr_method_name(method_name) {
    adg = std::make_shared<ADG>(num_robots);
    // confirmed_picks_by_genre.resize(NUM_GENRE, 0);
    // genre_finish_steps.resize(NUM_GENRE, 0);
    // mobile_manager =
    //     std::make_shared<MobileTaskManager>(num_robots, num_pickers,
    //     map_name);
    // picker_manager =
    //     std::make_shared<PickTaskManager>(num_pickers, NUM_GENRE, map_name);

    output_filename = target_output_filename;
    numRobots = adg->numRobots();
    agent_finish_time.resize(numRobots, -1);
    agents_finish.resize(numRobots, false);
    agent_finish_sim_step.resize(numRobots, -1);
    // auto name_mapping = adg->createRobotIDToStartIndexMaps();
    // robotIDTOStartIndex = name_mapping.first;
    // startIndexToRobotID = name_mapping.second;

    startTimers.resize(numRobots);
}

void ADG_Server::saveStats(int selector_wait_t, int capacity) {
    std::ifstream infile(output_filename);
    bool exist = infile.good();
    infile.close();
    if (!exist) {
        ofstream addHeads(output_filename);
        addHeads << "Num of agent,Num of Finished Tasks" << endl;
        addHeads.close();
    }
    ofstream stats(output_filename, std::ios::app);
    stats << "," << numRobots << endl;
    stats.close();
    std::cout << "Statistics written to " << output_filename << std::endl;
}

string getRobotsLocation(int look_ahead_dist) {
    // Return message as a JSON string
    json result_message = {};

    std::lock_guard<std::mutex> guard(globalMutex);
    std::cout << "Get robot location query received! lookahead dist: "
              << look_ahead_dist << std::endl;

    server_ptr->curr_robot_states =
        server_ptr->adg->computeCommitCut(look_ahead_dist);
    std::cout << "Commit cut number: " << server_ptr->curr_robot_states.size()
              << ", total number of robots: " << server_ptr->numRobots
              << std::endl;

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
    std::cout << "retrieving last actions" << std::endl;
    set<int> new_finished_tasks = server_ptr->adg->updateFinishedTasks();
    std::cout << "retrieved last actions" << std::endl;

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

// std::vector<std::vector<std::tuple<int, int, double>>> getGoals(
//     int goal_num = 1) {
//     std::lock_guard<std::mutex> guard(globalMutex);
//     std::vector<std::vector<std::tuple<int, int, double>>> new_goals;
//     if (not server_ptr->adg->initialized) {
//         return new_goals;
//     }
//     std::cout << "Query goals" << std::endl;
//     std::vector<std::deque<std::shared_ptr<MobileRobotTask>>> new_tasks;
//     server_ptr->mobile_manager->getTask(server_ptr->robots_location,
//     new_tasks); assert(new_tasks.size() == server_ptr->numRobots); std::cout
//     << "Get all goals" << std::endl;

//     server_ptr->curr_mobile_tasks = new_tasks;
//     new_goals.resize(server_ptr->numRobots);
//     assert(new_tasks.size() == server_ptr->numRobots);
//     std::unordered_map<std::pair<int, int>, std::vector<int>, pair_hash>
//         all_targets;
//     server_ptr->current_robots_goal_type.resize(server_ptr->numRobots);
//     std::vector<bool> transfer_agents(server_ptr->numRobots, false);
//     for (int agent_id = 0; agent_id < new_tasks.size(); agent_id++) {
//         std::cout << "agent_id: " << agent_id << std::endl;
//         if (server_ptr->adg->isLastActUnfinishedTransfer(agent_id)) {
//             transfer_agents[agent_id] = true;
//             auto tmp_loc = server_ptr->adg->getLastLoc(agent_id);
//             std::swap(tmp_loc.first, tmp_loc.second);
//             all_targets[tmp_loc].push_back(agent_id);
//         } else if (not new_tasks[agent_id].empty()) {
//             for (auto& task : new_tasks[agent_id]) {
//                 std::pair<int, int> tmp_loc = task->get_goal_position();
//                 all_targets[tmp_loc].push_back(agent_id);
//             }
//         }
//     }

//     // Step 2: Resolve conflicts
//     unordered_set<pair<int, int>, pair_hash> assigned_locations;
//     for (auto& [target, agent_ids] : all_targets) {
//         if (agent_ids.size() == 1) {
//             int agent = agent_ids[0];
//             if (transfer_agents[agent]) {
//                 server_ptr->current_robots_goal_type[agent] = NONE;
//             } else {
//                 server_ptr->current_robots_goal_type[agent] =
//                     server_ptr->curr_mobile_tasks[agent].front()->status;
//             }
//             insertNewGoal(agent, target, new_goals);
//             assigned_locations.insert(target);
//             printf("AGENT %d: Only goal locs::(%d, %d)\n", agent,
//             target.first,
//                    target.second);

//         } else {
//             // Find the closest agent
//             int closest_agent = -1;
//             int min_dist = numeric_limits<int>::max();

//             for (int id : agent_ids) {
//                 if (transfer_agents[id]) {
//                     server_ptr->current_robots_goal_type[id] = NONE;
//                     closest_agent = id;
//                     break;
//                 }
//                 auto curr_pos = server_ptr->curr_robot_states[id].position;
//                 int dist = twoPointDistance(
//                     std::make_pair(static_cast<int>(curr_pos.second),
//                                    static_cast<int>(curr_pos.first)),
//                     target);
//                 if (dist < min_dist) {
//                     min_dist = dist;
//                     closest_agent = id;
//                     server_ptr->current_robots_goal_type[closest_agent] =
//                         server_ptr->curr_mobile_tasks[closest_agent]
//                             .front()
//                             ->status;
//                 }
//             }

//             // Assign real target to the closest agent
//             insertNewGoal(closest_agent, target, new_goals);
//             assigned_locations.insert(target);
//             printf("AGENT %d: Closest goal locs::(%d, %d)\n", closest_agent,
//                    target.first, target.second);

//             // Assign neighbor locations to others
//             for (int id : agent_ids) {
//                 if (id == closest_agent)
//                     continue;
//                 server_ptr->current_robots_goal_type[id] = NONE;
//                 // pair<int, int> new_target =
//                 //
//                 server_ptr->mobile_manager->user_map.findNeighborPos(assigned_locations,
//                 // target);
//                 pair<int, int> new_target =
//                     server_ptr->mobile_manager->user_map.findNeighborPos(
//                         assigned_locations, target);

//                 insertNewGoal(id, new_target, new_goals);
//                 assigned_locations.insert(new_target);
//                 printf("AGENT %d: nearby goal locs::(%d, %d)\n", id,
//                        new_target.first, new_target.second);
//             }
//         }
//     }

//     for (int agent_id = 0; agent_id < new_tasks.size(); agent_id++) {
//         printf("agent_id: %d\n", agent_id);
//         if (new_tasks[agent_id].empty()) {
//             // If no task can be found, use the current locations
//             robotState curr_robot_loc =
//             server_ptr->curr_robot_states[agent_id]; if
//             (server_ptr->mobile_manager->user_map.isStation(
//                     curr_robot_loc.position)) {
//                 std::cout << "Robot is at station now: "
//                           << curr_robot_loc.position.second << ", "
//                           << curr_robot_loc.position.first << ", "
//                           << curr_robot_loc.orient;
//             }

//             int x, y;
//             if (assigned_locations.contains(
//                     std::make_pair(curr_robot_loc.position.second,
//                                    curr_robot_loc.position.first)) or
//                 server_ptr->mobile_manager->user_map.isStation(
//                     std::make_pair(curr_robot_loc.position.second,
//                                    curr_robot_loc.position.first)) or
//                 curr_robot_loc.position.first >= 60) {
//                 std::pair<int, int> random_loc =
//                     server_ptr->mobile_manager->user_map.findRandomPos(
//                         assigned_locations);
//                 x = random_loc.first;
//                 y = random_loc.second;
//             } else {
//                 x = static_cast<int>(curr_robot_loc.position.second);
//                 y = static_cast<int>(curr_robot_loc.position.first);
//             }

//             // x = static_cast<int>(curr_robot_loc.position.second);
//             // y = static_cast<int>(curr_robot_loc.position.first);
//             new_goals[agent_id].emplace_back(x, y, curr_robot_loc.orient);
//             assigned_locations.insert(std::make_pair(x, y));
//             server_ptr->current_robots_goal_type[agent_id] = NONE;
//             printf("Random goal locs::(%d, %d)\n",
//                    std::get<0>(new_goals[agent_id].front()),
//                    std::get<1>(new_goals[agent_id].front()));
//         }
//     }

//     std::unordered_map<std::pair<int, int>, std::vector<int>, pair_hash>
//         duplicate_targets;
//     for (int agent_id = 0; agent_id < new_goals.size(); agent_id++) {
//         std::vector<std::tuple<int, int, double>> tmp_agent =
//             new_goals[agent_id];
//         for (auto& tmp_element : tmp_agent) {
//             std::pair<int, int> tmp_loc{std::get<0>(tmp_element),
//                                         std::get<1>(tmp_element)};
//             if (duplicate_targets.find(tmp_loc) != duplicate_targets.end()) {
//                 std::cerr << "Duplicate agent found! Agent " << agent_id
//                           << ". Duplicate goal location: " << tmp_loc.first
//                           << ", " << tmp_loc.second << std::endl;
//                 string skip_info;
//                 std::cout << "Previous seen as: " << std::endl;
//                 for (auto prev_agent : duplicate_targets[tmp_loc]) {
//                     std::cerr
//                         << "Agent " << prev_agent << ", Act type: "
//                         << server_ptr->current_robots_goal_type[prev_agent]
//                         << std::endl;
//                 }
//                 std::cerr << "Act type: "
//                           << server_ptr->current_robots_goal_type[agent_id]
//                           << ", Continue? y/n" << std::endl;
//                 std::cin >> skip_info;
//             } else {
//                 duplicate_targets[tmp_loc].push_back(agent_id);
//             }
//         }
//     }

//     for (int agent_id = 0; agent_id < new_goals.size(); agent_id++) {
//         std::cout << "agent_id " << agent_id << "with action status: "
//                   << server_ptr->current_robots_goal_type[agent_id]
//                   << std::endl;
//         std::cout << "Goal locations: "
//                   << std::get<0>(new_goals[agent_id].front()) << ","
//                   << std::get<1>(new_goals[agent_id].front()) << std::endl;
//     }

//     // DEBUG only, replace the first location
//     // std::tuple<int, int, double> tmp_goal{2, 108, 0};
//     // new_goals[0].clear();
//     // new_goals[0].push_back(tmp_goal);
//     return new_goals;
// }

// typedef std::tuple<int, int, int> PickData;

// std::vector<PickData> getPickerTask() {
//     std::lock_guard<std::mutex> guard(globalMutex);
//     std::cout << "Request new picker task!" << std::endl;
//     std::vector<std::shared_ptr<PickerTask>> all_tasks;
//     server_ptr->picker_manager->getTask(all_tasks);
//     assert(not all_tasks.empty());
//     std::vector<PickData> all_pick_tasks;
//     for (auto& task : all_tasks) {
//         all_pick_tasks.emplace_back(task->obj_position.first,
//                                     task->obj_position.second, task->id);
//     }
//     return all_pick_tasks;
// }

// int getGenreID(int agent_id) {
//     int genre_id = 0;
//     genre_id = (agent_id / 4) * 2;
//     if (agent_id % 4 == 1 or agent_id % 4 == 2) {
//         genre_id += 1;
//     }
//     return genre_id;
// }

void updateStats(double total_wait_time, int capacity) {
    std::cout << "update the stats with: " << total_wait_time
              << ", and capacity: " << capacity << std::endl;
    server_ptr->total_wait_time = total_wait_time;
    server_ptr->transport_capacity = capacity;
}

void closeServer(rpc::server& srv) {
    std::cout << "close server called" << std::endl;
    server_ptr->saveStats(server_ptr->total_wait_time,
                          server_ptr->transport_capacity);
    srv.stop();
}

// bool confirmPickerTask(int agent_id, int task_id, int sim_step) {
//     assert(task_id != -1);
//     std::lock_guard<std::mutex> guard(globalMutex);
//     // std::cout << "send confirmation to agent " << agent_id << " with task
//     id
//     // " << task_id << std::endl;
//     int genre_id;
//     bool status =
//         server_ptr->picker_manager->confirmTask(agent_id, task_id, genre_id);
//     if (status) {
//         server_ptr->total_confirmed_picks++;
//         server_ptr->confirmed_picks_by_genre[genre_id] += 1;
//         if (server_ptr->confirmed_picks_by_genre[genre_id] >= MAX_TASKS) {
//             server_ptr->genre_finish_steps[genre_id] = sim_step;
//         }
//         if (server_ptr->total_confirmed_picks >= 8 * MAX_TASKS) {
//             std::cout << "Total finished tasks: "
//                       << server_ptr->mobile_manager->total_finished_tasks_
//                       << std::endl;
//             std::cout << "Total confirmed picks: "
//                       << server_ptr->total_confirmed_picks << std::endl;
//             for (int i = 0; i < NUM_GENRE; i++) {
//                 std::cout << "Finish step for genre is: "
//                           << server_ptr->genre_finish_steps[i] / 10.0
//                           << ", total tasks finished is: "
//                           << server_ptr->confirmed_picks_by_genre[i]
//                           << std::endl;
//             }
//             return true;
//         }
//     }
//     return false;
// }

// int requestMobileTask(int picker_id, std::pair<int, int> target_pos) {
//     std::lock_guard<std::mutex> guard(globalMutex);
//     std::cout << "Request mobile task! " << "with x: " << target_pos.first
//               << ", y: " << target_pos.second << std::endl;
//     int new_task_id = server_ptr->mobile_manager->insertPickerTask(
//         picker_id, target_pos.first, target_pos.second);
//     std::cout << "New mobile task id: " << new_task_id << std::endl;
//     return new_task_id;
// }

// void confirmMobileTask();

// std::string getScenConfigName() {
//     std::string target_path = server_ptr->curr_method_name + "/" +
//                               server_ptr->curr_map_name + "/" +
//                               std::to_string(server_ptr->numRobots) + "/" +
//                               server_ptr->curr_scen_name;
//     return target_path;
// }

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
    server_ptr->step_cnt++;
    return server_ptr->adg->getPlan(Robot_ID);
}

void updateSimFinishTime(std::string& robot_id_str, int sim_step) {
    int robot_id = server_ptr->adg->startIndexToRobotID[robot_id_str];
    if (server_ptr->agent_finish_sim_step[robot_id] < 0) {
        server_ptr->agent_finish_sim_step[robot_id] = sim_step;
        server_ptr->latest_arr_sim_step = sim_step;
    }
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
            ("num_pickers", po::value<int>()->required(), "number of picker robots in server")
            ("port_number,n", po::value<int>()->default_value(8080), "rpc port number")
            ("output_file,o", po::value<string>()->default_value("stats.csv"), "output statistic filename")
            ("map_file,m", po::value<string>()->default_value("empty-8-8.map"), "map filename")
            ("scen_file,s", po::value<string>()->default_value("empty-8-8-random-1"), "scen filename")
            ("method_name", po::value<string>()->default_value("PBS"), "method we used")
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

    if (vm.count("path_file")) {
        filename = vm["path_file"].as<string>();
    }
    // std::cout << "Solving for path name: " << filename << std::endl;
    std::string out_filename = vm["output_file"].as<string>();
    server_ptr = std::make_shared<ADG_Server>(
        vm["num_robots"].as<int>(), vm["num_pickers"].as<int>(), out_filename,
        vm["map_file"].as<string>(), vm["scen_file"].as<string>(),
        vm["method_name"].as<string>());

    int port_number = vm["port_number"].as<int>();
    rpc::server srv(port_number);  // Setup the server to listen on the
                                   // specified port number
    srv.bind("receive_update",
             &actionFinished);  // Bind the function to the server
    srv.bind("init", &init);
    srv.bind("get_location", &getRobotsLocation);
    // srv.bind("get_goals", &getGoals);
    srv.bind("add_plan", &addNewPlan);
    srv.bind("update", &update);
    // srv.bind("get_config", &getScenConfigName);
    srv.bind("update_finish_agent", &updateSimFinishTime);

    // functions with picker
    // srv.bind("get_picker_task", &getPickerTask);
    // srv.bind("confirm_picker_task", &confirmPickerTask);
    // srv.bind("request_mobile_robot", &requestMobileTask);

    // srv.bind("closeServer", [&srv](int wait_t, int capacity) {
    //     closeServer(srv, wait_t, capacity);
    // });
    srv.bind("update_stats", &updateStats);
    srv.bind("closeServer", [&srv]() { closeServer(srv); });
    srv.run();  // Start the server, blocking call
    // try {
    //     rpc::server srv(port_number);  // Setup the server to listen on the
    //     specified port number srv.bind("receive_update", &receive_update); //
    //     Bind the function to the server srv.bind("init", &init);
    //     srv.bind("get_location", &getRobotsLocation);
    //     srv.bind("get_goals", &getGoals);
    //     srv.bind("add_plan", &addNewPlan);
    //     srv.bind("update", &update);
    //     srv.bind("get_config", &getScenConfigName);
    //     srv.bind("update_finish_agent", &updateSimFinishTime);
    //     srv.bind("closeServer", [&srv]() {
    //         closeServer(srv);
    //     });
    //     srv.run();  // Start the server, blocking call
    // } catch (...) {
    //     // Catch any other exceptions
    //     std::cerr << "Fail to starting the server for scen "<<
    //     vm["scen_file"].as<string>() << " at port number: " << port_number <<
    //     std::endl;
    // }
    return 0;
}
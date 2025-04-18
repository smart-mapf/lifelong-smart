#include "ADG_server.h"

#include <random_task.h>

std::vector<std::chrono::steady_clock::time_point> startTimers; // Start times for each robot
std::shared_ptr<ADG_Server> server_ptr = nullptr;
    // =======================

ADG_Server::ADG_Server(int num_robots,
    int num_pickers,
    std::string& target_output_filename, 
    std::string map_name, 
    std::string scen_name,
    std::string method_name): curr_map_name(map_name), curr_scen_name(scen_name), curr_method_name(method_name)
 {
    adg = std::make_shared<ADG> (num_robots);
    mobile_manager = std::make_shared<MobileTaskManager>(num_robots, num_pickers, map_name);
    picker_manager = std::make_shared<PickTaskManager>(num_pickers, NUM_GENRE, map_name);

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

void ADG_Server::saveStats() {
    std::ifstream infile(output_filename);
    bool exist = infile.good();
    infile.close();
    if (!exist) {
        ofstream addHeads(output_filename);
        addHeads << "steps finish sim,sum of steps finish sim,time finish sim,sum finish time,original plan cost," <<
                 "#type-2 edges,#type-1 edges,#Nodes,"
                 "#Move,#Rotate,#Consecutive Move,#Agent pair," <<
                 "instance name,number of agent" << endl;
        addHeads.close();
    }
    ofstream stats(output_filename, std::ios::app);
    stats << latest_arr_sim_step << "," << std::accumulate(agent_finish_sim_step.begin(), agent_finish_sim_step.end(), 0) << "," <<
        *(std::max_element(agent_finish_time.begin(), agent_finish_time.end())) << "," <<
        std::accumulate(agent_finish_time.begin(), agent_finish_time.end(), 0.0) << "," <<
        raw_plan_cost << "," << adg->adg_stats.type2EdgeCount << "," <<
        adg->adg_stats.type1EdgeCount << "," << adg->adg_stats.totalNodes << "," <<
        adg->adg_stats.moveActionCount << "," << adg->adg_stats.rotateActionCount << "," <<
        adg->adg_stats.consecutiveMoveSequences << "," << static_cast<int>(adg->adg_stats.conflict_pairs.size())
        << "," << numRobots << endl;
    stats.close();
    // std::cout << "ADG statistics written to " << output_filename << std::endl;
    json result = {
        {"steps finish sim", latest_arr_sim_step},
        {"sum of steps finish sim", std::accumulate(agent_finish_sim_step.begin(), agent_finish_sim_step.end(), 0)},
        {"time finish sim", *(std::max_element(agent_finish_time.begin(), agent_finish_time.end()))},
        {"sum finish time", std::accumulate(agent_finish_time.begin(), agent_finish_time.end(), 0.0)},
        {"original plan cost", raw_plan_cost},
        {"#type-2 edges", adg->adg_stats.type2EdgeCount},
        {"#type-1 edges", adg->adg_stats.type1EdgeCount},
        {"#Nodes", adg->adg_stats.totalNodes},
        {"#Move", adg->adg_stats.moveActionCount},
        {"#Rotate", adg->adg_stats.rotateActionCount},
        {"#Consecutive Move", adg->adg_stats.consecutiveMoveSequences},
        {"#Agent pair", static_cast<int>(adg->adg_stats.conflict_pairs.size())},
        {"number of agent", numRobots}
    };

    // Output the JSON
    std::cout << result.dump() << std::endl; 
}


std::vector<std::pair<double, double>> getRobotsLocation(int look_ahead_dist) {
    std::lock_guard<std::mutex> guard(globalMutex);
    std::cout << "Get robot location query received! lookahead dist: " << look_ahead_dist << std::endl;
    // if (server_ptr->adg->initialized and not server_ptr->debug_set_flag) {
    //     server_ptr->debug_set_flag = true;
    // } else {
    //     return {};
    // }
    server_ptr->curr_robot_states = server_ptr->adg->computeCommitCut(look_ahead_dist);
    assert(static_cast<int>(server_ptr->curr_robot_states.size()) == server_ptr->numRobots);
    std::cout << "Commit cut number: " << server_ptr->curr_robot_states.size() << ", total number of robots: "
        << server_ptr->numRobots << std::endl;
    std::vector<std::pair<double, double>> robots_location;
    for (auto& robot : server_ptr->curr_robot_states)
    {
        if (server_ptr->flipped_coord) {
            robots_location.emplace_back(robot.position.second, robot.position.first);
        } else {
            robots_location.emplace_back(robot.position.first, robot.position.second);
        }
    }

    // TODO@jingtian: set a task as finished when it is enqueued
    std::vector< std::unordered_set<int> > task_status;
    std::cout << "retrieving last actions" << std::endl;
    auto success_status = server_ptr->adg->getFinishedTasks(task_status);
    std::cout << "retrieved last actions" << std::endl;
    if (success_status) {
        for (int i = 0; i < server_ptr->numRobots; i++) {
            if (not server_ptr->curr_mobile_tasks[i].empty() and task_status[i].contains(server_ptr->curr_mobile_tasks[i].front()->id)) {
                server_ptr->mobile_manager->finishTask(i, server_ptr->curr_mobile_tasks[i].front());
            }
        }
    }
    server_ptr->robots_location = robots_location;
    return robots_location;
}

void addNewPlan(std::vector<std::vector<std::tuple<int, int, double>>>& new_plan) {
    // x, y and time
    std::lock_guard<std::mutex> guard(globalMutex);
    int agent_id = 0;
    std::vector<std::vector<Step>> raw_plan;
    for (auto& plan : new_plan)
    {
        std::vector<Point> points;
        std::vector<Step> tmp_plan;
        for (auto& step : plan)
        {
            points.emplace_back(std::get<1>(step), std::get<0>(step), std::get<2>(step));
        }
        processAgentActions(points, tmp_plan, -1, agent_id);
        agent_id++;
        raw_plan.push_back(tmp_plan);
    }

    std::vector<std::vector<Action>> plans;
    plans = processActions(raw_plan, server_ptr->flipped_coord);
#ifdef  DEBUG
    std::cout << "Finish action process, plan size: " << plans.size() << std::endl;
#endif

    for (int i = 0; i < static_cast<int>(plans.size()); i++) {
        if (server_ptr->curr_mobile_tasks[i].empty() or
            server_ptr->curr_mobile_tasks[i].front()->status == DONE) {
            std::cout << "No plan" << std::endl;
            continue;
        }
        Action tmp_act;
        tmp_act.robot_id = i;
        // @jingtian Note: change action start time, to be consistent with the continuous case
        if (plans[i].empty()) {
            tmp_act.time = 0;
            tmp_act.orientation = 0;
            tmp_act.nodeID = 0;
        } else {
            tmp_act.time = plans[i].back().time + 1;
            tmp_act.orientation = plans[i].back().orientation;
            tmp_act.nodeID = plans[i].back().nodeID + 1;
        }
        std::pair<int, int> tmp_loc = server_ptr->curr_mobile_tasks[i].front()->get_goal_position();

        tmp_act.start = tmp_loc;
        tmp_act.goal = tmp_loc;
        if (server_ptr->flipped_coord) {
            double tmp = tmp_act.goal.first;
            tmp_act.goal.first = tmp_act.goal.second;
            tmp_act.goal.second = tmp;
            std::swap(tmp_act.start.first, tmp_act.start.second);
        }

        if (server_ptr->curr_mobile_tasks[i].front()->status == MobileAction::DELIVER) {
            // If station
            tmp_act.type = 'S';
        } else {
            tmp_act.type = 'P';
        }
        tmp_act.task_id = server_ptr->curr_mobile_tasks[i].front()->id;
        plans[i].push_back(tmp_act);
    }
    // showActionsPlan(plans);
    server_ptr->adg->addMAPFPlan(plans);
    // server_ptr->adg->showGraph();
    // for (int id = 0; id < server_ptr->numRobots; id++)
    // {
    //     std::pair<double, double> curr_pos = server_ptr->adg->getRobotPosition(id);
    //     server_ptr->task_manager_ptr->isAgentFinished(id, curr_pos);
    // }
#ifdef DEBUG
    std::cout << "Finish add plan" << std::endl;
#endif

}

std::string actionFinished(std::string& RobotID, int node_ID) {
    std::lock_guard<std::mutex> guard(globalMutex);
    if (not server_ptr->adg->initialized) {
        std::cerr << "ADG_Server::ADG_Server: server_ptr is not initialized" << std::endl;
        exit(-1);
        return "None";
    }
    int Robot_ID = server_ptr->adg->startIndexToRobotID[RobotID];
    bool status_update = server_ptr->adg->updateFinishedNode(Robot_ID, node_ID);
    std::pair<double, double> curr_pos = server_ptr->adg->getRobotPosition(Robot_ID);
    // if (server_ptr->adg->isTaskNode(Robot_ID, node_ID)) {
    //     std::cout << "receive confirmation for robot id: " << Robot_ID << ", at node id: "<< node_ID << "with current position at: " <<
    //     curr_pos.first << ", " << curr_pos.second << std::endl;
    //     if (not server_ptr->curr_tasks[Robot_ID].empty()) {
    //         if (server_ptr->curr_tasks[Robot_ID].front()->status == true) {
    //             server_ptr->task_manager_ptr->setTask(Robot_ID, server_ptr->curr_tasks[Robot_ID].front(), false);
    //         }
    //     }
    // }

    // printf("goal::(%f, %f), curr_pos::(%f, %f)\n", server_ptr->adg->getActionGoal(Robot_ID, node_ID).first,
    //     server_ptr->adg->getActionGoal(Robot_ID, node_ID).second, curr_pos.first, curr_pos.second);
    // if (server_ptr->task_manager_ptr->isAgentFinished(Robot_ID, curr_pos)) {
    //     auto endTime = std::chrono::steady_clock::now();
    //     auto diff = endTime - startTimers[Robot_ID];
    //     double duration = std::chrono::duration_cast<std::chrono::duration<double>>(diff).count();
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


std::vector<std::vector<std::tuple<int, int, double>>> getGoals(int goal_num=1)
{
    std::lock_guard<std::mutex> guard(globalMutex);
    std::vector<std::vector<std::tuple<int, int, double>>> new_goals;
    if (not server_ptr->adg->initialized) {
        return new_goals;
    }
    std::vector<std::deque<std::shared_ptr<MobileRobotTask>>> new_tasks;
    assert(new_tasks.size() == server_ptr->numRobots);
    server_ptr->mobile_manager->getTask(server_ptr->robots_location, new_tasks);
    server_ptr->curr_mobile_tasks = new_tasks;
    new_goals.resize(server_ptr->numRobots);
    assert(new_tasks.size() == server_ptr->numRobots);
    std::unordered_set<std::pair<int, int>, pair_hash> all_targets;
    for (int agent_id = 0; agent_id < new_tasks.size(); agent_id++) {
        std::cout << "agent_id: " << agent_id << std::endl;
        if (not new_tasks[agent_id].empty()) {
            for (auto& task : new_tasks[agent_id]) {

                std::pair<int, int> tmp_loc = task->get_goal_position();
                int tmp_x = tmp_loc.first;
                int tmp_y = tmp_loc.second;
                if (not server_ptr->flipped_coord)
                {
                    tmp_x = tmp_loc.second;
                    tmp_y = tmp_loc.first;
                }
                all_targets.insert(std::make_pair(tmp_x, tmp_y));
                std::cout << "task goal location: " << tmp_x << ", " << tmp_y << ", status: " << task->status << std::endl;
            }
        }
    }

    for (int agent_id = 0; agent_id < new_tasks.size(); agent_id++) {
        printf("agent_id: %d\n", agent_id);
        if (new_tasks[agent_id].empty()) {
            // If no task can be found, use the current locations
            // std::cout << "Size of the curr robot states: " << server_ptr->curr_robot_states.size() << std::endl;
            assert(server_ptr->curr_robot_states.size() == server_ptr->numRobots);
            robotState curr_robot_loc = server_ptr->curr_robot_states[agent_id];
            if (server_ptr->mobile_manager->user_map.isStation(curr_robot_loc.position)) {
                std::cout << "Robot is at station now: " << curr_robot_loc.position.second << ", " <<
                    curr_robot_loc.position.first << ", " << curr_robot_loc.orient;
            }

            int x, y;
            // std::pair<int, int> random_loc = server_ptr->mobile_manager->user_map.findRandomPos(all_targets);
            // x = random_loc.first;
            // y = random_loc.second;

            if (all_targets.contains(curr_robot_loc.position) or
                server_ptr->mobile_manager->user_map.isStation(curr_robot_loc.position)) {
                std::pair<int, int> random_loc = server_ptr->mobile_manager->user_map.findRandomPos(all_targets);
                x = random_loc.first;
                y = random_loc.second;
            } else {
                x = static_cast<int>(curr_robot_loc.position.second);
                y = static_cast<int>(curr_robot_loc.position.first);
            }


            // x = static_cast<int>(curr_robot_loc.position.second);
            // y = static_cast<int>(curr_robot_loc.position.first);
            new_goals[agent_id].emplace_back(x, y, curr_robot_loc.orient);
            all_targets.insert(std::make_pair(x, y));
            printf("Random goal locs::(%d, %d)\n", std::get<0>(new_goals[agent_id].front()), std::get<1>(new_goals[agent_id].front()));

        } else {
            std::cout << "Size of the new tasks for agent " << agent_id << ", is: " << new_tasks[agent_id].size() << std::endl;
            for (auto& task : new_tasks[agent_id]) {
                std::pair<int, int> tmp_loc = task->get_goal_position();

                int tmp_x = tmp_loc.first;
                int tmp_y = tmp_loc.second;
                if (not server_ptr->flipped_coord)
                {
                    tmp_x = tmp_loc.second;
                    tmp_y = tmp_loc.first;
                }
                new_goals[agent_id].emplace_back(tmp_x, tmp_y, task->goal_orient);
                // all_targets.insert(std::make_pair(tmp_x, tmp_y));
            }
            printf("Assigned goal locs::(%d, %d)\n", std::get<0>(new_goals[agent_id].front()), std::get<1>(new_goals[agent_id].front()));

        }
    }

    return new_goals;
}

// std::vector<bool> checkGoalsStatus(std::vector<std::vector<std::tuple<int, int, double>>>& new_goals)
// {
//     ;
// }

// pos_x, pos_y, task_id
typedef std::tuple<int, int, int> PickData;

std::vector< PickData > getPickerTask() {
    std::lock_guard<std::mutex> guard(globalMutex);
    std::cout << "Request new picker task!" << std::endl;
    std::vector<std::shared_ptr<PickerTask>> all_tasks;
    server_ptr->picker_manager->getTask(all_tasks);
    assert(not all_tasks.empty());
    std::vector< PickData > all_pick_tasks;
    for (auto& task : all_tasks) {
        all_pick_tasks.emplace_back(task->obj_position.first, task->obj_position.second, task->id);
    }
    return all_pick_tasks;
}

void confirmPickerTask(int agent_id, int task_id) {
    assert(task_id != -1);
    std::lock_guard<std::mutex> guard(globalMutex);
    // std::cout << "send confirmation to agent " << agent_id << " with task id " << task_id << std::endl;
    server_ptr->picker_manager->confirmTask(agent_id, task_id);
}

int requestMobileTask(int picker_id, std::pair<int, int> target_pos) {
    std::lock_guard<std::mutex> guard(globalMutex);
    std::cout << "Request mobile task! " << "with x: " << target_pos.first << ", y: " << target_pos.second << std::endl;
    int new_task_id = server_ptr->mobile_manager->insertPickerTask(picker_id,
        target_pos.first, target_pos.second);
    std::cout << "New mobile task id: " << new_task_id << std::endl;
    return new_task_id;
}

// void confirmMobileTask();

std::string getScenConfigName()
{
    std::string target_path = server_ptr->curr_method_name + "/" + server_ptr->curr_map_name + "/" + std::to_string(server_ptr->numRobots) + "/" + server_ptr->curr_scen_name;
    return target_path;
}


std::vector<std::tuple<std::string, int, double, std::string, std::pair<double, double>, std::pair<double, double>, int>> update(std::string RobotID) {
    std::lock_guard<std::mutex> guard(globalMutex);
    if (not server_ptr->adg->initialized or not server_ptr->adg->get_initial_plan) {
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

void updateSimFinishTime(std::string& robot_id_str, int sim_step)
{
    int robot_id = server_ptr->adg->startIndexToRobotID[robot_id_str];
    if (server_ptr->agent_finish_sim_step[robot_id] < 0) {
        server_ptr->agent_finish_sim_step[robot_id] = sim_step;
        server_ptr->latest_arr_sim_step = sim_step;
    }
}

void closeServer(rpc::server& srv)
{
    server_ptr->saveStats();
    srv.stop();
}

int main(int argc, char **argv) {
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
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
    server_ptr = std::make_shared<ADG_Server>(vm["num_robots"].as<int>(), vm["num_pickers"].as<int>(), out_filename, vm["map_file"].as<string>(),
        vm["scen_file"].as<string>(), vm["method_name"].as<string>());

    int port_number = vm["port_number"].as<int>();
    rpc::server srv(port_number);  // Setup the server to listen on the specified port number
    srv.bind("receive_update", &actionFinished);  // Bind the function to the server
    srv.bind("init", &init);
    srv.bind("get_location", &getRobotsLocation);
    srv.bind("get_goals", &getGoals);
    srv.bind("add_plan", &addNewPlan);
    srv.bind("update", &update);
    srv.bind("get_config", &getScenConfigName);
    srv.bind("update_finish_agent", &updateSimFinishTime);

    // functions with picker
    srv.bind("get_picker_task", &getPickerTask);
    srv.bind("confirm_picker_task", &confirmPickerTask);
    srv.bind("request_mobile_robot", &requestMobileTask);
    srv.bind("closeServer", [&srv]() {
        closeServer(srv);
    });
    srv.run();  // Start the server, blocking call
    // try {
    //     rpc::server srv(port_number);  // Setup the server to listen on the specified port number
    //     srv.bind("receive_update", &receive_update);  // Bind the function to the server
    //     srv.bind("init", &init);
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
    //     std::cerr << "Fail to starting the server for scen "<< vm["scen_file"].as<string>() << " at port number: " << port_number << std::endl;
    // }
    return 0;
}
#include "ADG_server.h"

#include <random_task.h>

std::vector<std::chrono::steady_clock::time_point> startTimers; // Start times for each robot
    // =======================

ADG_Server::ADG_Server(int num_robots,
    std::string& target_output_filename, 
    std::string map_name, 
    std::string scen_name,
    std::string method_name): curr_map_name(map_name), curr_scen_name(scen_name), curr_method_name(method_name)
 {
    adg = std::make_shared<ADG> (num_robots);
    task_manager_ptr = std::make_shared<RandomTask>(num_robots, map_name);
    output_filename = target_output_filename;
    numRobots = adg->numRobots();
    agent_finish_time.resize(numRobots, -1);
    agents_finish.resize(numRobots, false);
    agent_finish_sim_step.resize(numRobots, -1);
    // auto name_mapping = adg->createRobotIDToStartIndexMaps();
    // robotIDTOStartIndex = name_mapping.first;
    // startIndexToRobotID = name_mapping.second;

    outgoingEdgesByRobot.resize(numRobots);
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


std::shared_ptr<ADG_Server> server_ptr = nullptr;

std::vector<std::pair<double, double>> getRobotsLocation(int look_ahead_dist) {
    std::cout << "Get robot location query received!" << std::endl;
    std::vector<std::pair<double, double>> robots_location = server_ptr->adg->computeCommitCut(look_ahead_dist);
    if (server_ptr->flipped_coord)
    {
        for (auto& robot : robots_location)
        {
            std::swap(robot.first, robot.second);
        }
    }

    std::vector<std::pair<double, double>> goal_location;
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
        processAgentActions(points, tmp_plan, server_ptr->adg->curr_commit[agent_id].orient, agent_id);
        agent_id++;
        raw_plan.push_back(tmp_plan);
    }

    std::vector<std::vector<Action>> plans;
    plans = processActions(raw_plan, server_ptr->flipped_coord);
#ifdef  DEBUG
    std::cout << "Finish action process, plan size: " << plans.size() << std::endl;
#endif

    for (int i = 0; i < static_cast<int>(plans.size()); i++) {
        if (server_ptr->curr_tasks[i].empty()) {
            std::cout << "curr task is empty" << std::endl;
            continue;
        } else if (server_ptr->curr_tasks[i].front() == nullptr) {
            std::cout << "Invalid task" << std::endl;
            continue;
        } else if (plans[i].empty()) {
            std::cout << "No plan" << std::endl;
            continue;
        }
        std::cout << "original plan size: " << plans[i].size() << std::endl;
        Action tmp_act;
        tmp_act.robot_id = (int) i;
        // @jingtian Note: change action start time, to be consistent with the continuous case
        tmp_act.time = plans[i].back().time;
        tmp_act.start = server_ptr->curr_tasks[i].front()->goal_position;
        tmp_act.goal = server_ptr->curr_tasks[i].front()->obj_position;
        if (server_ptr->flipped_coord) {
            double tmp = tmp_act.goal.first;
            tmp_act.goal.first = tmp_act.goal.second;
            tmp_act.goal.second = tmp;
            std::swap(tmp_act.start.first, tmp_act.start.second);
        }
        tmp_act.orientation = 0;
        tmp_act.nodeID = 0;
        if (server_ptr->curr_tasks[i].front()->flag == 0) {
            // If station
            tmp_act.type = 'S';
        } else {
            tmp_act.type = 'P';
        }
        plans[i].push_back(tmp_act);
    }
    showActionsPlan(plans);
    server_ptr->adg->addMAPFPlan(plans);
    // for (int id = 0; id < server_ptr->numRobots; id++)
    // {
    //     std::pair<double, double> curr_pos = server_ptr->adg->getRobotPosition(id);
    //     server_ptr->task_manager_ptr->isAgentFinished(id, curr_pos);
    // }
#ifdef DEBUG
    std::cout << "Finish add plan" << std::endl;
#endif

}

std::string receive_update(std::string& RobotID, int node_ID) {
    std::lock_guard<std::mutex> guard(globalMutex);
    if (not server_ptr->adg->initialized) {
        std::cerr << "ADG_Server::ADG_Server: server_ptr is not initialized" << std::endl;
        exit(-1);
        return "None";
    }
    int Robot_ID = server_ptr->adg->startIndexToRobotID[RobotID];
    bool status_update = server_ptr->adg->updateFinishedNode(Robot_ID, node_ID);
    std::pair<double, double> curr_pos = server_ptr->adg->getRobotPosition(Robot_ID);
    printf("goal::(%f, %f), curr_pos::(%f, %f)\n", server_ptr->adg->getActionGoal(Robot_ID, node_ID).first,
        server_ptr->adg->getActionGoal(Robot_ID, node_ID).second, curr_pos.first, curr_pos.second);
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

    //
    // server_ptr->all_agents_finished = true;
    // for (auto status: server_ptr->agents_finish) {
    //     if (not status) {
    //         server_ptr->all_agents_finished = false;
    //     }
    // }
    // if (server_ptr->all_agents_finished) {
    //     return "exit";
    // }
    // if (server_ptr->agents_finish[Robot_ID]) {
    //     return "end";
    // }
    return "None";
}

void init(std::string RobotID) {
    std::lock_guard<std::mutex> guard(globalMutex);
    printf("robot ID: %s\n", RobotID.c_str());
    bool is_all_init = server_ptr->adg->createRobotIDToStartIndexMaps(RobotID);
    if (is_all_init)
    {
        ;
        std::cout << "All agents are initialized\n";
    }
    // int Robot_ID = server_ptr->startIndexToRobotID[RobotID];
// #ifdef DEBUG
//     std::cerr << "TMP::Receive init request from agent " << RobotID << " with: " << Robot_ID << std::endl;
//     if (Robot_ID == DEBUG_AGENT) {
//         std::cerr << "Receive init request from agent " << Robot_ID << std::endl;
//         exit(0);
//     }
// #endif
    // startTimers[Robot_ID] = std::chrono::steady_clock::now();
    // return server_ptr->adg->getPlan(Robot_ID);
}

void addNewGoals(std::vector<std::vector<std::tuple<int, int, double>>>& new_goals)
{
    ;
}

std::vector<std::vector<std::tuple<int, int, double>>> getGoals(int goal_num=1)
{
    std::lock_guard<std::mutex> guard(globalMutex);
    std::vector<std::vector<std::tuple<int, int, double>>> new_goals;
    if (not server_ptr->adg->initialized) {
        return new_goals;
    }
    std::vector<bool> task_status;
    std::cout << "retrieving last actions" << std::endl;
    auto success_status = server_ptr->adg->getLastAction(task_status);
    std::cout << "retrieved last actions" << std::endl;
    if (success_status) {
        for (int i = 0; i < task_status.size(); i++) {
            if (task_status[i] ) {
                server_ptr->task_manager_ptr->setTask(i, server_ptr->curr_tasks[i].front(), false);
            }
        }
    }
    std::vector<std::deque<std::shared_ptr<Task>>> new_tasks;
    server_ptr->task_manager_ptr->getTask(new_tasks);
    server_ptr->curr_tasks = new_tasks;
    new_goals.resize(server_ptr->numRobots);
    std::cout << "get new tasks" << std::endl;
    for (int agent_id = 0; agent_id < new_tasks.size(); agent_id++) {
        for (auto& task : new_tasks[agent_id]) {
            int tmp_x = task->goal_position.first;
            int tmp_y = task->goal_position.second;
            // if (server_ptr->flipped_coord)
            // {
            //     tmp_x = task->goal_position.second;
            //     tmp_y = task->goal_position.first;
            // }

            // printf("goal locs::(%d, %d)\n", tmp_x, tmp_y);
            new_goals[agent_id].emplace_back(tmp_x, tmp_y, task->goal_orient);
        }
    }
    return new_goals;
}

// std::vector<bool> checkGoalsStatus(std::vector<std::vector<std::tuple<int, int, double>>>& new_goals)
// {
//     ;
// }

std::string getScenConfigName()
{
    std::string target_path = server_ptr->curr_method_name + "/" + server_ptr->curr_map_name + "/" + std::to_string(server_ptr->numRobots) + "/" + server_ptr->curr_scen_name;
    return target_path;
}


std::vector<std::tuple<std::string, int, double, std::string, std::pair<double, double>, std::pair<double, double>>> update(std::string RobotID) {
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
#ifdef DEBUG
    if (Robot_ID == DEBUG_AGENT)
        std::cerr << "Receive update request from agent " << Robot_ID << std::endl;
#endif
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
    // if (server_ptr->all_agents_finished) {
    //     std::cout << "Finish all actions, exiting..." << std::endl;
    // } else {
    //     std::cout << "BUG::exiting without finish all actions" << std::endl;
    // }
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
//            ("path_file,p", po::value<string>()->default_value("../data/maze-32-32-4_paths.txt"), "input file for path")
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
    server_ptr = std::make_shared<ADG_Server>(vm["num_robots"].as<int>(), out_filename, vm["map_file"].as<string>(),
        vm["scen_file"].as<string>(), vm["method_name"].as<string>());

    int port_number = vm["port_number"].as<int>();
    rpc::server srv(port_number);  // Setup the server to listen on the specified port number
    srv.bind("receive_update", &receive_update);  // Bind the function to the server
    srv.bind("init", &init);
    srv.bind("get_location", &getRobotsLocation);
    srv.bind("get_goals", &getGoals);
    srv.bind("add_plan", &addNewPlan);
    srv.bind("update", &update);
    srv.bind("get_config", &getScenConfigName);
    srv.bind("update_finish_agent", &updateSimFinishTime);
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
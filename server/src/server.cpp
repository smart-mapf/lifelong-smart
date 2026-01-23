#include "ExecutionManager.h"

namespace rpc_api {

/// @cond DOXYGEN_EXCLUDE
shared_ptr<ExecutionManager> em = nullptr;
std::mutex globalMutex;

void freezeSimulationIfNecessary() {
    lock_guard<mutex> guard(globalMutex);
    em->freezeSimulationIfNecessary();
}

// Return the simulation freeze status
bool isSimulationFrozen() {
    lock_guard<mutex> guard(globalMutex);
    return em->isSimulationFrozen();
}

string actionFinished(string &robot_id_str, int node_ID) {
    lock_guard<mutex> guard(globalMutex);
    return em->actionFinished(robot_id_str, node_ID);
}

void init(string RobotID, tuple<int, int> init_loc) {
    lock_guard<mutex> guard(globalMutex);
    em->init(RobotID, init_loc);
}

void closeServer(rpc::server &srv) {
    spdlog::info("Closing server at port {}", em->getRPCPort());
    em->saveStats();
    srv.close_sessions();
    srv.stop();
    spdlog::info("Server closed successfully.");
}
SIM_PLAN obtainActionsFromADG(string RobotID) {
    lock_guard<mutex> guard(globalMutex);
    return em->obtainActionsFromADG(RobotID);
}

bool simStatus() {
    lock_guard<mutex> guard(globalMutex);
    return em->stopSimulation();
}

void updateSimStep() {
    lock_guard<mutex> guard(globalMutex);
    em->updateSimStep();
}
void recordStatsPerTick() {
    lock_guard<mutex> guard(globalMutex);
    em->recordStatsPerTick();
}
/// @endcond

/// @brief Get the current locations of all robots
/// @return A string representing the robots' locations
/// The format of the string can be defined as needed, e.g. JSON
string getRobotsLocation() {
    lock_guard<mutex> guard(globalMutex);
    return em->getRobotsLocation();
}

void addNewPlan(string &new_plan_json_str) {
    lock_guard<mutex> guard(globalMutex);
    em->addNewPlan(new_plan_json_str);
}

bool isInitialized() {
    lock_guard<mutex> guard(globalMutex);
    return em->isADGInitialized();
}

bool invokePlanner() {
    lock_guard<mutex> guard(globalMutex);
    return em->invokePlanner();
}

}  // namespace rpc_api

/// @cond DOXYGEN_EXCLUDE
int main(int argc, char **argv) {
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
            ("heuristic_type", po::value<string>()->default_value("basic"), "heuristic type: basic, landmark")
            ("num_landmarks", po::value<int>()->default_value(20), "number of landmarks for landmark heuristic (default: 20)")
            ("landmark_selection", po::value<string>()->default_value("workstation+endpt_corners"), "landmark selection strategy: random, workstation+endpt_corners")
            ("save_heuristics_table", po::value<bool>()->default_value(false), "save heuristics table or not")
            ("task_assigner_type", po::value<string>()->default_value("windowed"), "windowed, one_goal, distinct_one_goal")
            ("task_file", po::value<string>()->default_value(""), "task file for the robots (default: empty, meaning random tasks)")
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

    rpc_api::em = make_shared<ExecutionManager>(vm);

    // Set up logger
    auto console_logger = spdlog::default_logger()->clone("ExecutionManager");
    spdlog::set_default_logger(console_logger);

    // Setup the server to listen on the specified port number
    rpc::server srv(port_number);

    // Bind the function to the server
    srv.bind("receive_update", &rpc_api::actionFinished);
    srv.bind("init", &rpc_api::init);
    srv.bind("is_initialized", &rpc_api::isInitialized);
    srv.bind("get_location", &rpc_api::getRobotsLocation);
    srv.bind("add_plan", &rpc_api::addNewPlan);
    srv.bind("obtain_actions", &rpc_api::obtainActionsFromADG);
    srv.bind("update_sim_step", &rpc_api::updateSimStep);
    srv.bind("invoke_planner", &rpc_api::invokePlanner);
    srv.bind("close_server", [&srv]() { rpc_api::closeServer(srv); });
    srv.bind("freeze_simulation_if_necessary",
             &rpc_api::freezeSimulationIfNecessary);
    srv.bind("is_simulation_frozen", &rpc_api::isSimulationFrozen);
    srv.bind("sim_status", &rpc_api::simStatus);
    srv.bind("record_stats_per_tick", &rpc_api::recordStatsPerTick);
    srv.run();  // Start the server, blocking call

    return 0;
}

/// @endcond
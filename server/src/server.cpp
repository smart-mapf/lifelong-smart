#include "ExecutionManager.h"

/**
 * @namespace RPC API functions for external communication with the
 * MAPF planner and the executor (the robots).
 */
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

/**
 * @brief Get the latest MAPF instance, newly finished task IDs, and
 * initialization status.
 *
 * @return A JSON string with the following schema.
 * @code{.json}
 * {
 *   "initialized": true,
 *   "mapf_instance": {
 *     "starts": [ State, ... ],
 *     "goals":  [ [ Task, ... ], ... ]
 *   },
 *   "new_finished_tasks": [ 0, 1, 2, 3 ]
 * }
 * @endcode
 * - `initialized` indicates whether the system has completed initialization.
 * - `mapf_instance` contains the current MAPF problem state:
 *   - `starts` is a list of `State` objects describing the current start states
 * of all agents:
 *     - Each `State` has the structure:
 *       @code{.json}
 *       {
 *         "location": 42,
 *         "timestep": 10,
 *         "orientation": 1
 *       }
 *       @endcode
 *     - `location` is the flattened cell ID.
 *     - `timestep` is the start timestep, usually `0`.
 *     - `orientation` is encoded as:
 *       - `0`: East
 *       - `1`: North
 *       - `2`: West
 *       - `3`: South
 *   - `goals` is a list of task lists, where each task list corresponds to a
 *     robot:
 *     - Each `Task` object has the structure:
 *       @code{.json}
 *       {
 *         "id": 0,
 *         "location": 84,
 *         "task_wait_time": 0,
 *         "orientation": -1
 *       }
 *       @endcode
 *     - `id` is the unique task identifier.
 *     - `location` is the flattened cell ID of the task location.
 *     - `task_wait_time` is the number of timesteps the agent should wait at
 *        the task location, usually `0`.
 *     - `orientation` is the required orientation at the task location, with
 *       `-1` indicating no constraint.
 * - `new_finished_tasks` is a list of task IDs completed since the last query.
 *
 */
string getRobotsLocation() {
    lock_guard<mutex> guard(globalMutex);
    return em->getRobotsLocation();
}

/**
 * @brief Add a new MAPF plan to the ADG.
 *
 * @details
 * This function takes a JSON string representing a new MAPF plan and adds it
 * to the ADG (Action Decision Graph). If necessary, it utilizes the backup
 * planner to ensure the plan is integrated correctly.
 *
 * @param new_plan_json_str A JSON string with a new MAPF plan with the
 * following schema:
 * @code{.json}
 * {
 *   "success": true,
 *   "plan": [ [(row, col, timestep, task_id), ... ], ... ],
 *   "congested": false,
 *   "stats": { ... }
 * }
 * @endcode
 * - `plan`: a list of lists of tuples, where each outer list
 *   corresponds to a single robot's path. Each inner list contains tuples of
 *   the form `(row, col, timestep, task_id)`:
 *   - `row`: The row index of the robot's position on the grid.
 *   - `col`: The column index of the robot's position on the grid.
 *   - `timestep`: The timestep at which the robot should be at the specified
 * position.
 *   - `task_id`: The task identifier, or `-1` if no task is associated.
 * - `success`: indicates whether the planner successfully produced
 *   collision-free paths for all robots. If `false`, the backup planner is
 * invoked.
 * - `congested` **[Optional]**: indicates whether congestion was detected,
 *   typically defined as more than half of the robots making no progress.
 * - `stats` **[Optional]**: contains additional planner statistics recorded
 *   by LSMART.
 */
void addNewPlan(string &new_plan_json_str) {
    lock_guard<mutex> guard(globalMutex);
    em->addNewPlan(new_plan_json_str);
}

/**
 * @brief Check if the system has been initialized.
 * @returns a boolean indicating whether the system has been initialized.
 */
bool isInitialized() {
    lock_guard<mutex> guard(globalMutex);
    return em->isADGInitialized();
}

/**
 * @brief Determine whether to invoke the planner.
 * @returns a boolean indicating whether the planner should be invoked. In
 * practice, the MAPF planner can have a while loop checking for this
 * condition to decide whether to request a new MAPF instance from the system.
 */
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
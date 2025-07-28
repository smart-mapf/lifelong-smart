/*driver.cpp
 * Solve a MAPF instance on 2D grids.
 */
#include <rpc/client.h>
#include <rpc/rpc_error.h>

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <memory>
#include <string>

#include "common.h"
#include "instance.h"
#include "pbs.h"
#include "pibt.h"
#include "task_assigner.h"

/* Main function */
int main(int argc, char **argv) {
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
            ("help", "produce help message")

            // params for the input instance and experiment settings
            ("map,m", po::value<string>()->default_value("random-32-32-20.map"), "input file for map")
            // ("agents,a", po::value<string>()->default_value(""), "input file for agents")

            // ("heuristic,h",
            //  po::value<string>()->default_value("./data/sortation_large/heuristic/sortation_large-random-1.heuristic"),
            //  "heuristic for agents")
            ("output,o", po::value<string>()->default_value("./output/output.txt"), "output file for schedule")
            ("statistic,c", po::value<string>()->default_value("./output/output.csv"),
             "output file for statistic result")
            ("agentNum,k", po::value<int>()->default_value(1), "number of agents")
            // ("agentIdx", po::value<string>()->default_value(""), "customize the indices of the agents (e.g., \"0,1\")")
            ("seed,d", po::value<int>()->default_value(0), "random seed")
            ("solver,s", po::value<int>()->default_value(0),
             "SPS solver, choose from 0 for BAS, and 1 for BCS")
            ("outputPaths", po::value<string>(), "output file for paths")
            ("partialExpansion,p", po::value<bool>()->default_value(1), "enable partial expansion")
            ("cutoffTime,t", po::value<double>()->default_value(2), "cutoff time (seconds)")
            ("screen", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
            ("simulation_window", po::value<double>()->default_value(10), "simulation window (in seconds) for the planner")

            // params for instance generators
            // ("rows", po::value<int>()->default_value(0), "number of rows")
            // ("cols", po::value<int>()->default_value(0), "number of columns")
            // ("obs", po::value<int>()->default_value(0), "number of obstacles")
            // ("warehouseWidth", po::value<int>()->default_value(0),
            //  "width of working stations on both sides, for generating instances")

            // params for CBS
            // ("sipp", po::value<bool>()->default_value(false), "using sipp as the single agent solver")

            // params for rpc
            ("portNum", po::value<int>()->default_value(8080), "port number for the server");
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return 1;
    }

    po::notify(vm);
    /////////////////////////////////////////////////////////////////////////
    /// check the correctness and consistence of params
    //////////////////////////////////////////////////////////////////////

    int seed = vm["seed"].as<int>();
    srand(seed);

    // Set up logger
    auto console_logger = spdlog::default_logger()->clone("Planner");
    spdlog::set_default_logger(console_logger);

    int num_agents = vm["agentNum"].as<int>();
    int screen = vm["screen"].as<int>();
    double simulation_window = vm["simulation_window"].as<double>();
    int sps_solver_type = vm["solver"].as<int>();
    std::shared_ptr<Graph> graph =
        make_shared<Graph>(vm["map"].as<string>(), screen);
    std::shared_ptr<TaskAssigner> task_assigner =
        make_shared<TaskAssigner>(graph, screen, simulation_window, num_agents);

    // Backup solver
    int simulation_window_ts = static_cast<int>(simulation_window * V_MAX);
    PIBT pibt(graph, simulation_window_ts, screen, seed, num_agents);

    // Stats
    int n_mapf_calls = 0;        // number of MAPF calls
    int n_rule_based_calls = 0;  // number of rule-based calls

    // We assume the server is already running at this point.
    rpc::client client("127.0.0.1", vm["portNum"].as<int>());

    // Wait for the server to initialize
    int trial = 0;
    while (!client.call("is_initialized").as<bool>()) {
        if (screen > 0) {
            // printf("%d Waiting for server to initialize...\n", trial);
            trial++;
        }
    }

    while (true) {
        // Get the current simulation tick
        bool invoke_planner = client.call("invoke_planner").as<bool>();

        // Skip planning until the simulation step is a multiple of the
        // simulation window
        if (!invoke_planner) {
            continue;
        }

        string result_message = client.call("get_location").as<string>();

        auto result_json = json::parse(result_message);
        if (!result_json["initialized"].get<bool>()) {
            printf("Planner not initialized! Retrying\n");
            sleep(1);
            continue;
        }
        auto commit_cut =
            result_json["robots_location"]
                .get<std::vector<std::tuple<double, double, int>>>();
        auto new_finished_tasks_id =
            result_json["new_finished_tasks"].get<std::set<int>>();

        if (screen > 0) {
            // printf("Agent locations:\n");
            // for (auto loc : commit_cut) {
            //     printf("%f %f\n", loc.first, loc.second);
            // }

            printf("New finished tasks:\n");
            for (auto finish_task_id : new_finished_tasks_id) {
                printf("%d,", finish_task_id);
            }
            printf("\n");
        }

        //////////////////////////////////////////////////////////////////////
        /// load the instance
        //////////////////////////////////////////////////////////////////////
        std::shared_ptr<Instance> instance_ptr = std::make_shared<Instance>(
            graph, task_assigner, commit_cut, new_finished_tasks_id,
            vm["partialExpansion"].as<bool>(), sps_solver_type, screen,
            simulation_window);

        //////////////////////////////////////////////////////////////////////
        /// initialize the solver
        //////////////////////////////////////////////////////////////////////
        PBS pbs(instance_ptr, sps_solver_type, vm["cutoffTime"].as<double>(),
                screen);
        auto init_start_time = Time::now();
        // pbs.sipp_ptr->getHeuristic(vm["heuristic"].as<std::string>());
        auto init_end_time = Time::now();
        time_s debug_init_d = init_end_time - init_start_time;
        double debug_init_time = debug_init_d.count();
        // printf("Finish initialization the heuristic in %f seconds\n",
        //        debug_init_time);
        n_mapf_calls++;
        auto global_start_time = Time::now();
        bool pbs_success = pbs.solve(vm["output"].as<string>());
        // bool pbs_success = false;
        auto global_end_time = Time::now();
        std::chrono::duration<float> global_run_time =
            global_end_time - global_start_time;
        // printf("Runtime for MASS is: %f\n", global_run_time.count());
        if (screen > 0) {
            spdlog::info("Runtime for MASS is: {} seconds",
                         global_run_time.count());
        }
        vector<vector<tuple<int, int, double, int>>> new_mapf_plan;
        new_mapf_plan.resize(instance_ptr->num_of_agents);
        if (pbs_success) {
            // printf("Solution found!\n");
            if (screen > 0) {
                spdlog::info("MASS: Solution found!");
            }
            pbs.updateCost();
            new_mapf_plan = pbs.getTimedPath();
            pbs.clear();
            // if (vm.count("outputPaths"))
            //     pbs.saveTimedPath(vm["outputPaths"].as<std::string>());
            // pbs.savePath("durationPath.txt");
        } else {
            // printf("No solution found!\n");
            if (screen > 0) {
                spdlog::warn("MASS: No solution found, invoking PIBT!");
            }
            pibt.run(instance_ptr->getStartLocations(),
                     instance_ptr->task_assigner->getGoalLocations());
            n_rule_based_calls++;
            new_mapf_plan = pibt.getPaths();
            pibt.clear();
        }

        // Send new plan
        json stats = {{"n_mapf_calls", n_mapf_calls},
                      {"n_rule_based_calls", n_rule_based_calls}};
        json new_plan_json = {
            {"plan", new_mapf_plan},
            {"congested", false},
            {"stats", stats.dump()},

        };
        client.call("add_plan", new_plan_json.dump());

        // pbs.saveResults(pbs_success, vm["statistic"].as<string>(),
        //                 vm["map"].as<string>());
    }
}
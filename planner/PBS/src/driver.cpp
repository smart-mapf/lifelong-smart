/* Copyright (C) Jiaoyang Li
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Confidential
 * Written by Jiaoyang Li <jiaoyanl@usc.edu>, May 2020
 */

/*driver.cpp
 * Solve a MAPF instance on 2D grids.
 */
#include <rpc/client.h>
#include <rpc/rpc_error.h>

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

#include "PBS.h"

/* Main function */
int main(int argc, char** argv) {
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    // clang-format off
	desc.add_options()
		("help", "produce help message")

		// params for the input instance and experiment settings
		("map,m", po::value<string>()->required(), "input file for map")
		("output,o", po::value<string>(), "output file for statistics")
		("outputPaths", po::value<string>(), "output file for paths")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", po::value<double>()->default_value(3), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("stats", po::value<bool>()->default_value(false), "write to files some detailed statistics")
        ("portNum", po::value<int>()->default_value(8080), "port number for the server")
		("sipp", po::value<bool>()->default_value(1), "using SIPP as the low-level solver")
		("simulation_window", po::value<int>()->default_value(5),
         "The path planned every time should be >= this length")
        ("seed", po::value<int>()->default_value(0), "random seed");
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return 1;
    }

    po::notify(vm);

    int seed = vm["seed"].as<int>();
    srand(seed);  // Set the random seed for reproducibility

    ///////////////////////////////////////////////////////////////////////////
    int prev_last_task_id = 0;  // last task id from the previous iteration
    vector<Task> prev_goal_locs;
    set<int> finished_tasks_id;
    int screen = vm["screen"].as<int>();
    int simulation_window = vm["simulation_window"].as<int>();

    // Create a graph, heuristic will be computed only once in the graph
    Graph graph(vm["map"].as<string>(), screen);

    // We assume the server is already running at this point.
    rpc::client client("127.0.0.1", vm["portNum"].as<int>());
    // client.set_timeout(5000);  // in ms

    // Wait for the server to initialize
    int trial = 0;
    while (!client.call("is_initialized").as<bool>()) {
        if (screen > 0) {
            printf("%d Waiting for server to initialize...\n", trial);
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

        Instance instance(graph, prev_goal_locs, screen, prev_last_task_id,
                          simulation_window);
        instance.loadAgents(commit_cut, new_finished_tasks_id);
        PBS pbs(instance, vm["sipp"].as<bool>(), screen);
        // run
        double runtime = 0;
        bool success = false;
        double runtime_limit = vm["cutoffTime"].as<double>();
        int fail_count = 0;
        while (not success) {
            success = pbs.solve(runtime_limit);
            runtime_limit = runtime_limit * 2;
            if (success) {
                auto new_mapf_plan = pbs.getPaths();

                pbs.clearSearchEngines();
                json new_plan_json = {
                    {"plan", new_mapf_plan},
                    {"congested", false},
                };
                client.call("add_plan", new_plan_json.dump());
            } else {
                std::cerr << "No solution found!" << std::endl;
                fail_count++;
                pbs.clear();
                if (fail_count > 10) {
                    std::cerr << "Fail to find a solution after " << fail_count
                              << " attempts! Exiting." << std::endl;
                    // instance.saveInstance();
                    exit(-1);
                }
            }
        }
        // instance.printMap();
        // Update task id for the next iteration
        prev_last_task_id = instance.getTaskId();
        prev_goal_locs = instance.getGoalTasks();
        // sleep(0.5);
    }

    cout << "Planner finished!" << endl;
    return 0;
}
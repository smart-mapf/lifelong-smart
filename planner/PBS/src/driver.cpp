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

#include "Graph.h"
#include "Instance.h"
#include "PBS.h"
#include "TaskAssigner.h"
#include "common.h"

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

    // Set up logger
    auto console_logger = spdlog::default_logger()->clone("Planner");
    spdlog::set_default_logger(console_logger);

    ///////////////////////////////////////////////////////////////////////////
    // int prev_last_task_id = 0;  // last task id from the previous iteration
    // vector<Task> prev_goal_locs;
    set<int> finished_tasks_id;
    int screen = vm["screen"].as<int>();
    int simulation_window = vm["simulation_window"].as<int>();
    int num_agents = vm["agentNum"].as<int>();

    // Create a graph, heuristic will be computed only once in the graph
    auto graph = make_shared<Graph>(vm["map"].as<string>(), screen);
    auto task_assigner =
        make_shared<TaskAssigner>(graph, screen, simulation_window, num_agents);

    // Stats
    int n_mapf_calls = 0;        // number of MAPF calls
    int n_rule_based_calls = 0;  // number of rule-based calls

    // We must have enough empty locations for agents to "wait in queue".
    // Since we do not consider window in this planner, it is not okay for
    // agents to have the goal. Therefore when agent j tries to go to a
    // location which is the goal of agent i, it must go to an empty space
    // first.
    if (graph->nEmptyLocations() < num_agents) {
        spdlog::error("Not enough empty locations for agents to wait in queue. "
                      "Please increase the number of empty locations.");
        exit(-1);
    }

    // We assume the server is already running at this point.
    rpc::client client("127.0.0.1", vm["portNum"].as<int>());
    // client.set_timeout(5000);  // in ms

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

        // Obtain the MAPF instance
        if (!result_json.contains("mapf_instance")) {
            spdlog::error("PBS Driver: mapf_instance not found in the JSON "
                          "from server. Exit...");
            exit(1);
        }

        if (!result_json["mapf_instance"].contains("starts") ||
            !result_json["mapf_instance"].contains("goals")) {
            spdlog::error("PBS Driver: starts or goals not found in the "
                          "mapf_instance from server. Exit...");
            exit(1);
        }
        json mapf_instance = result_json["mapf_instance"];

        Instance instance(graph, task_assigner, screen, simulation_window);
        // instance.loadAgents(commit_cut, new_finished_tasks_id);
        instance.loadAgents(mapf_instance);
        PBS pbs(instance, vm["sipp"].as<bool>(), screen);
        // run
        double runtime = 0;
        bool success = false;
        double runtime_limit = vm["cutoffTime"].as<double>();
        int fail_count = 0;
        n_mapf_calls += 1;
        success = pbs.solve(runtime_limit);
        runtime_limit = runtime_limit * 2;
        vector<vector<tuple<int, int, double, int>>> new_mapf_plan;
        if (success) {
            new_mapf_plan = pbs.getPaths();

            pbs.clearSearchEngines();
            pbs.clear();

        } else {
            n_rule_based_calls += 1;
        }

        // Send new plan
        json stats = {{"n_mapf_calls", n_mapf_calls},
                      {"n_rule_based_calls", n_rule_based_calls}};
        json new_plan_json = {
            {"success", success},
            {"plan", new_mapf_plan},
            {"congested", congested(new_mapf_plan)},
            {"stats", stats.dump()},
        };
        client.call("add_plan", new_plan_json.dump());
    }

    cout << "Planner finished!" << endl;
    return 0;
}
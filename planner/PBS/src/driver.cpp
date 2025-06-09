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

// Return true if the timeout has exceeded the maximum number of attempts
bool rpc_timeout_handle(const rpc::timeout& e, int& timeouts,
                        int timeout_attempts, int screen) {
    timeouts++;
    if (screen > 0) {
        printf("%s. ", e.what());
    }

    if (timeouts > timeout_attempts) {
        if (screen > 0) {
            printf("Timeout exceeded %d attempts. Exiting...\n",
                   timeout_attempts);
        }
        return true;  // Exceeded maximum attempts
    }
    if (screen > 0) {
        printf("Retrying...\n");
    }
    return false;  // Still within the allowed attempts
}

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
        ("timeout_attempts", po::value<int>()->default_value(10),
         "number of attempts to get location from server before exiting")
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
    // load the instance
    int task_id = 0;
    vector<Task> prev_goal_locs;
    set<int> finished_tasks_id;
    int timeouts = 0;
    int timeout_attempts = vm["timeout_attempts"].as<int>();
    int screen = vm["screen"].as<int>();
    // We assume the server is already running at this point.
    rpc::client client("127.0.0.1", vm["portNum"].as<int>());
    client.set_timeout(100);  // set timeout to 1000 ms
    while (true) {
        string result_message;
        try {
            result_message = client.call("get_location", 5).as<string>();
        } catch (const rpc::timeout& e) {
            if (rpc_timeout_handle(e, timeouts, timeout_attempts, screen)) {
                break;  // Exit if maximum attempts exceeded
            }
            continue;
        }
        // printf("Result message: %s\n", result_message.c_str());
        auto result_json = json::parse(result_message);
        if (!result_json["initialized"].get<bool>()) {
            printf("Planner not initialized! Retrying\n");
            sleep(1);
            continue;
        }
        auto commit_cut = result_json["robots_location"]
                              .get<std::vector<std::pair<double, double>>>();
        auto new_finished_tasks_id =
            result_json["new_finished_tasks"].get<std::set<int>>();

        if (screen > 0) {
            printf("Agent locations:\n");
            for (auto loc : commit_cut) {
                printf("%f %f\n", loc.first, loc.second);
            }

            printf("New finished tasks:\n");
            for (auto task_id : new_finished_tasks_id) {
                printf("%d,", task_id);
            }
            printf("\n");
        }

        // TODO@jingtian: update this to get new instance
        Instance instance(vm["map"].as<string>(), screen);
        instance.task_id = task_id;
        instance.setGoalLocations(prev_goal_locs);
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
                try {
                    client.call("add_plan", new_mapf_plan);
                } catch (const rpc::timeout& e) {
                    if (rpc_timeout_handle(e, timeouts, timeout_attempts,
                                           screen)) {
                        break;  // Exit if maximum attempts exceeded
                    }
                    continue;
                }
            } else {
                std::cerr << "No solution found!" << std::endl;
                fail_count++;
                if (fail_count > 1000) {
                    std::cerr << "Fail to find a solution after " << fail_count
                              << " attempts! Exiting." << std::endl;
                    // instance.saveInstance();
                    exit(-1);
                }
            }
        }
        // instance.printMap();
        // Update task id for the next iteration
        task_id = instance.task_id;
        prev_goal_locs = instance.getGoalTasks();
        sleep(0.1);
    }

    cout << "Planner finished!" << endl;
    return 0;
}
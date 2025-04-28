/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, May 2020
*/

/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "PBS.h"
#include <rpc/client.h>


/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
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
		;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		cout << desc << endl;
		return 1;
	}

	po::notify(vm);

	srand((int)time(0));

	///////////////////////////////////////////////////////////////////////////
	// load the instance
	rpc::client client("127.0.0.1", vm["portNum"].as<int>());
	while (true) {
		auto commit_cut = client.call("get_location", 5).as<std::vector<std::pair<double, double>>>();
		printf("Agent locations:\n");
		for (auto loc: commit_cut)
		{
			printf("%f %f\n", loc.first, loc.second);
		}
		auto goals = client.call("get_goals", 1).as<std::vector<std::vector<std::tuple<int, int, double>>>>();
		printf("Goals:\n");
		for (auto goal: goals)
		{
			printf("%d %d\n", std::get<0> (goal[0]), std::get<1> (goal[0]));
		}
		if (commit_cut.empty() or goals.empty())
		{
			printf("Planner not initialized! Retrying\n");
			sleep(1);
			continue;
		}
		// TODO@jingtian: update this to get new instance
		Instance instance(vm["map"].as<string>());
		instance.loadAgents(commit_cut, goals);
		PBS pbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
		// run
		double runtime = 0;
		bool success = false;
    double runtime_limit = vm["cutoffTime"].as<double>();
    while (not success) {
	    success = pbs.solve(runtime_limit);
      runtime_limit = runtime_limit*2;
      if (success)
      {
        // if (vm.count("output"))
        // 	pbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
        // if (pbs.solution_found && vm.count("outputPaths"))
        // 	pbs.savePaths(vm["outputPaths"].as<string>());
        /*size_t pos = vm["output"].as<string>().rfind('.');      // position of the file extension
        string output_name = vm["output"].as<string>().substr(0, pos);     // get the name without extension
        cbs.saveCT(output_name); // for debug*/
        std::cout << "######################################" << std::endl;
        auto new_mapf_plan = pbs.getPaths();
        std::cout << "######################################" << std::endl;
  
        pbs.clearSearchEngines();
        client.call("add_plan", new_mapf_plan);
        break;
      }
      else {
        std::cerr << "No solution found!" << std::endl;
      }
    }
    // instance.printMap();
		// sleep(1);
	}


	return 0;

}
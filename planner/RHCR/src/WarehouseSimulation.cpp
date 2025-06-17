// Main API for python to run simulation

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "KivaSystem.h"
#include "SortingSystem.h"
#include "OnlineSystem.h"
#include "BeeSystem.h"
#include "ManufactureSystem.h"
#include "GreyOrangeSystem.h"
#include "ID.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>
#include "helper.h"
using json = nlohmann::json;

namespace py = pybind11;

/**
 * Run one round of warehouse simulation
 *
 *
 *
 * @param kwargs py dict with all the params, specifically:
 * 	   scenario       		: scenario (SORTING, KIVA, ONLINE, BEE)
 *     map				 	: input map json string
 *     task                 : input task file
 *     output			    : output folder name
 *     agentNum             : number of drives
 *     cutoffTime           : cutoff time (seconds)
 *     OverallCutoffTime    : overall cutoff time (seconds)
 *     seed                 : random seed
 *     screen               : screen option (0: none; 1: results; 2:all)
 *     solver	            : solver (LRA, PBS, WHCA, ECBS)
 *     id                   : independence detection
 *     single_agent_solver	: single-agent solver (ASTAR, SIPP)
 *     lazyP                : use lazy priority
 *     simulation_time      : run simulation
 *     simulation_window    : call the planner every simulation_window timesteps
 *     travel_time_window   : consider the traffic jams within the given window
 *     planning_window      : the planner outputs plans with first
 *     						: planning_window timesteps collision-free
 *     potential_function	: potential function (NONE, SOC, IC)
 *     potential_threshold  : potential threshold
 *     rotation           	: consider rotation
 *     robust               : k-robust (for now, only work for PBS)
 *     CAT                	: use conflict-avoidance table
 *     hold_endpoints     	: Hold endpoints from Ma et al, AAMAS 2017
 *     dummy_paths        	: Find dummy paths from Liu et al, AAMAS 2019
 *     prioritize_start    	: Prioritize waiting at start locations
 *     suboptimal_bound     : Suboptimal bound for ECBS
 *     log                	: save the search trees (and the priority trees)
 *     force_new_logdir     : force the program to create a new logdir
 *     save_result          : whether save the result (path, etc) to disk
 *     save_solver          : whether save the solver result (solver.csv) to
 *                            disk
 *     save_heuristics_table: whether save the heuristics table to disk
 *     stop_at_traffic_jam  : whether to stop at traffic jam

 * @return result (json string): summerized objective, measures, and any
 * ancillary data
 */
std::string run(const py::kwargs& kwargs) {
	if (kwargs["test"].cast<bool>())
	{
		// For testing
		json result = {
			{"throughput", 10},
			{"tile_usage", {0.0, 1.0, 2.0}},
			{"num_wait", {3.0, 4.0, 5.0}},
			{"finished_task_len", {6.0, 7.0, 8.0}},
			{"tile_usage_mean", 1.0},
			{"tile_usage_std", 0.67},
			{"num_wait_mean", 4.0},
			{"num_wait_std", 0.67},
			{"finished_len_mean", 7.0},
			{"finished_len_std", 0.67}
		};

		return result.dump(4);
	}

    // Default variables
    if (!kwargs.contains("planning_window"))
    {
        kwargs["planning_window"] = INT_MAX / 2;
        // cout << kwargs["planning_window"].cast<int>() << endl;
    }

    if (!kwargs.contains("left_w_weight"))
    {
        kwargs["left_w_weight"] = 1.0;
    }

    if (!kwargs.contains("right_w_weight"))
    {
        kwargs["right_w_weight"] = 1.0;
    }

    if (!kwargs.contains("OverallCutoffTime"))
    {
        kwargs["OverallCutoffTime"] = INT_MAX / 2;
    }

    namespace po = boost::program_options;
    clock_t start_time = clock();
	json result;
	result["finish"] = false; // status code to false by default

    // check params
    if (kwargs["hold_endpoints"].cast<bool>() or kwargs["dummy_paths"].cast<bool>())
    {
        if (kwargs["hold_endpoints"].cast<bool>() and kwargs["dummy_paths"].cast<bool>())
        {
            std::cerr << "Hold endpoints and dummy paths cannot be used simultaneously" << endl;
            exit(-1);
        }
        if (kwargs["simulation_window"].cast<int>() != 1)
        {
            std::cerr << "Hold endpoints and dummy paths can only work when the simulation window is 1" << endl;
            exit(-1);
        }
        if (kwargs["planning_window"].cast<int>() < INT_MAX / 2)
        {
            std::cerr << "Hold endpoints and dummy paths cannot work with planning windows" << endl;
            exit(-1);
        }
    }

    // make dictionary
    bool force_new_logdir = kwargs["force_new_logdir"].cast<bool>();
	boost::filesystem::path dir(kwargs["output"].cast<std::string>() +"/");

    // Remove previous dir is necessary.
    if (boost::filesystem::exists(dir) && force_new_logdir)
    {
        boost::filesystem::remove_all(dir);
    }

    if (kwargs["log"].cast<bool>() ||
        kwargs["save_heuristics_table"].cast<bool>() ||
        kwargs["save_result"].cast<bool>() ||
        kwargs["save_solver"].cast<bool>())
	{
        boost::filesystem::create_directories(dir);
		boost::filesystem::path dir1(kwargs["output"].cast<std::string>() + "/goal_nodes/");
		boost::filesystem::path dir2(kwargs["output"].cast<std::string>() + "/search_trees/");
		boost::filesystem::create_directories(dir1);
		boost::filesystem::create_directories(dir2);
	}


	if (kwargs["scenario"].cast<string>() == "KIVA")
	{
		KivaGrid G;
		G.screen = kwargs["screen"].cast<int>();
        G.hold_endpoints = kwargs["hold_endpoints"].cast<bool>();
	    G.useDummyPaths = kwargs["dummy_paths"].cast<bool>();
        G._save_heuristics_table = kwargs["save_heuristics_table"].cast<bool>();
        G.rotation_time = kwargs["rotation_time"].cast<int>();
		if (!G.load_map_from_jsonstr(
                kwargs["map"].cast<std::string>(),
                kwargs["left_w_weight"].cast<double>(),
                kwargs["right_w_weight"].cast<double>()))
			return result.dump(4);
		MAPFSolver* solver = helper::set_solver(G, kwargs);
		KivaSystem system(G, *solver);
		helper::set_parameters(system, kwargs);
        system.start_time = start_time;
		G.preprocessing(system.consider_rotation,
						kwargs["output"].cast<std::string>());
		result = system.simulate(kwargs["simulation_time"].cast<int>());
		result["finish"] = true; // Change status code
        double runtime = (double)(clock() - start_time)/ CLOCKS_PER_SEC;
		cout << "Overall runtime: " << runtime << " seconds." << endl;
        result["cpu_runtime"] = runtime;
		return result.dump(4); // Dump to string with indent 4
	}
    else if (kwargs["scenario"].cast<string>() == "MANUFACTURE")
    {
        ManufactureGrid G;
		G.screen = kwargs["screen"].cast<int>();
        if (kwargs["hold_endpoints"].cast<bool>() ||
            kwargs["dummy_paths"].cast<bool>())
        {
            std::cerr << "Hold endpoints and dummy paths are not supported for manufacturing scenario" << endl;
            exit(-1);
        }
        G._save_heuristics_table = kwargs["save_heuristics_table"].cast<bool>();
        G.n_station_types = kwargs["n_station_types"].cast<int>();
        G.station_wait_time = kwargs["station_wait_times"].cast<vector<int>>();
        G.rotation_time = kwargs["rotation_time"].cast<int>();
        assert(G.station_wait_time.size() == G.n_station_types);
        if (!G.load_map_from_jsonstr(kwargs["map"].cast<std::string>()))
        {
            return result.dump(4);
        }
        MAPFSolver* solver = helper::set_solver(G, kwargs);
        solver->consider_task_wait = true;
		ManufactureSystem system(G, *solver);
		helper::set_parameters(system, kwargs);
        system.start_time = start_time;
		G.preprocessing(system.consider_rotation,
                        kwargs["output"].cast<std::string>());
		result = system.simulate(kwargs["simulation_time"].cast<int>());
        result["finish"] = true; // Change status code
        double runtime = (double)(clock() - start_time)/ CLOCKS_PER_SEC;
		cout << "Overall runtime: " << runtime << " seconds." << endl;
        result["cpu_runtime"] = runtime;
		return result.dump(4);
    }
    else if (kwargs["scenario"].cast<string>() == "GREYORANGE")
	{
		GreyOrangeGrid G;
		G.screen = kwargs["screen"].cast<int>();
        G.hold_endpoints = kwargs["hold_endpoints"].cast<bool>();
	    G.useDummyPaths = kwargs["dummy_paths"].cast<bool>();
        G._save_heuristics_table = kwargs["save_heuristics_table"].cast<bool>();
        G.rotation_time = kwargs["rotation_time"].cast<int>();
        if (!G.load_map_from_jsonstr(kwargs["map"].cast<std::string>()))
            return result.dump(4);
		MAPFSolver* solver = helper::set_solver(G, kwargs);
        solver->consider_task_wait = true;
        string agents_file = kwargs["agents"].cast<string>();
        string task_file = kwargs["task"].cast<string>();
		GreyOrangeSystem system(G, *solver, task_file, agents_file);
		helper::set_parameters(system, kwargs);
        system.start_time = start_time;
		G.preprocessing(system.consider_rotation, "map");
		result = system.simulate(kwargs["simulation_time"].cast<int>());
        result["finish"] = true; // Change status code
        double runtime = (double)(clock() - start_time)/ CLOCKS_PER_SEC;
		cout << "Overall runtime: " << runtime << " seconds." << endl;
        result["cpu_runtime"] = runtime;
		return result.dump(4);
	}
	else if (kwargs["scenario"].cast<string>() == "SORTING")
	{
		SortingGrid G;
		G.screen = kwargs["screen"].cast<int>();
        G.hold_endpoints = kwargs["hold_endpoints"].cast<bool>();
	    G.useDummyPaths = kwargs["dummy_paths"].cast<bool>();
        G._save_heuristics_table = kwargs["save_heuristics_table"].cast<bool>();
        G.rotation_time = kwargs["rotation_time"].cast<int>();
		if (!G.load_map_from_jsonstr(
                kwargs["map"].cast<std::string>(),
                kwargs["left_w_weight"].cast<double>(),
                kwargs["right_w_weight"].cast<double>()))
			return result.dump(4);
		MAPFSolver* solver = helper::set_solver(G, kwargs);

        // Read in packages
        std::string package_mode;
        vector<int> packages;
        vector<double> package_dist_weight;
        std::tie(package_mode, packages, package_dist_weight) = helper::setup_packages(kwargs);

        // Read in chute mapping
        map<int, vector<int>> chute_mapping_int = helper::setup_chute_mapping(kwargs);

        // Read in task assignment policy
        std::string task_assignment_cost;
        vector<double> task_assignment_params;
        std::tie(task_assignment_cost, task_assignment_params) = helper::setup_task_assign(kwargs);

		SortingSystem system(G, *solver, chute_mapping_int, package_mode,
            packages, package_dist_weight, task_assignment_cost,
            task_assignment_params);
		helper::set_parameters(system, kwargs);
        system.start_time = start_time;
		G.preprocessing(system.consider_rotation,
						kwargs["output"].cast<std::string>());
		result = system.simulate(kwargs["simulation_time"].cast<int>());
		result["finish"] = true; // Change status code
        double runtime = (double)(clock() - start_time)/ CLOCKS_PER_SEC;
		cout << "Overall runtime: " << runtime << " seconds." << endl;
        result["cpu_runtime"] = runtime;
		return result.dump(4); // Dump to string with indent 4
	}
    // *********** Note: currrently unsupported, TODO for future **********
	// else if (kwargs["scenario"].cast<string>() == "ONLINE")
	// {
	// 	OnlineGrid G;
	// 	if (!G.load_map(kwargs["map"].cast<std::string>()))
	// 		return py::make_tuple(-1);
	// 	MAPFSolver* solver = helper::set_solver(G, kwargs);
	// 	OnlineSystem system(G, *solver);
	// 	assert(!system.hold_endpoints);
	// 	assert(!system.useDummyPaths);
	// 	helper::set_parameters(system, kwargs);
	// 	G.preprocessing(system.consider_rotation);
	// 	system.simulate(kwargs["simulation_time"].cast<int>());
	// 	return py::make_tuple(0);
	// }
	// else if (kwargs["scenario"].cast<string>() == "BEE")
	// {
	// 	BeeGraph G;
	// 	if (!G.load_map(kwargs["map"].cast<std::string>()))
	// 		return py::make_tuple(-1);
	// 	MAPFSolver* solver = helper::set_solver(G, kwargs);
	// 	BeeSystem system(G, *solver);
	// 	assert(!system.hold_endpoints);
	// 	assert(!system.useDummyPaths);
	// 	helper::set_parameters(system, kwargs);
	// 	G.preprocessing(kwargs["tcastk"].cast<std::string>(), system.consider_rotation);
	// 	system.load_task_assignments(kwargs["tcastk"].cast<std::string>());
	// 	system.simulate();
	// 	double runtime = (double)(clock() - start_time)/ CLOCKS_PER_SEC;
	// 	cout << "Overall runtime:			" << runtime << " seconds." << endl;
	// 	// cout << "	Reading from file:		" << G.loading_time + system.loading_time << " seconds." << endl;
	// 	// cout << "	Preprocessing:			" << G.preprocessing_time << " seconds." << endl;
	// 	// cout << "	Writing to file:		" << system.saving_time << " seconds." << endl;
	// 	cout << "Makespan:		" << system.get_makespan() << " timesteps." << endl;
	// 	cout << "Flowtime:		" << system.get_flowtime() << " timesteps." << endl;
	// 	cout << "Flowtime lowerbound:	" << system.get_flowtime_lowerbound() << " timesteps." << endl;
	// 	auto flower_ids = system.get_missed_flower_ids();
	// 	cout << "Missed tcastks:";
	// 	for (auto id : flower_ids)
	// 		cout << " " << id;
	// 	cout << endl;
	// 	// cout << "Remaining tcastks: " << system.get_num_of_remaining_tcastks() << endl;
	// 	cout << "Objective: " << system.get_objective() << endl;
	// 	std::ofstream output;
	// 	output.open(kwargs["output"].cast<std::string>() + "/MAPF_results.txt", std::ios::out);
	// 	output << "Overall runtime: " << runtime << " seconds." << endl;;
	// 	output << "Makespan: " << system.get_makespan() << " timesteps." << endl;
	// 	output << "Flowtime: " << system.get_flowtime() << " timesteps." << endl;
	// 	output << "Flowtime lowerbound: " << system.get_flowtime_lowerbound() << " timesteps." << endl;
	// 	output << "Missed tcastks:";
	// 	for (auto id : flower_ids)
	// 		output << " " << id;
	// 	output << endl;
	// 	output << "Objective: " << system.get_objective() << endl;
	// 	output.close();
    //     return py::make_tuple(0);
	// }
	// *************************************************************
	else
	{
		cout << "Scenario " << kwargs["scenario"].cast<string>() << "does not exist!" << endl;
		return result.dump(4);
	}

    return result.dump(4);
}

string playground(){
	std::string json_string = R"(
	{
		"pi": 3.141,
		"happy": true
	}
	)";
	json ex1 = json::parse(json_string);

	cout << ex1["pi"] << endl;

	return ex1.dump();
}


PYBIND11_MODULE(warehouse_sim, m) {
	// optional module docstring
    // m.doc() = ;

    m.def("playground", &playground, "Playground function to test everything");
    // m.def("add", &add, py::arg("i")=0, py::arg("j")=1);
    m.def("run", &run, "Function to run warehouse simulation");
}
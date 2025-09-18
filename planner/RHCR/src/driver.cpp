#include "KivaSystem.h"
#include "ID.h"
#include "SMARTSystem.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>


void set_parameters(BasicSystem& system, const boost::program_options::variables_map& vm)
{
	system.outfile = vm["output"].as<std::string>();
	system.screen = vm["screen"].as<int>();
	system.log = vm["log"].as<bool>();
	system.num_of_drives = vm["agentNum"].as<int>();
	system.time_limit = vm["cutoffTime"].as<int>();
    system.overall_time_limit = vm["OverallCutoffTime"].as<int>();
	system.simulation_window = vm["simulation_window"].as<int>();
	system.planning_window = vm["planning_window"].as<int>();
	system.travel_time_window = vm["travel_time_window"].as<int>();
	system.consider_rotation = vm["rotation"].as<bool>();
	system.k_robust = vm["robust"].as<int>();
	system.hold_endpoints = vm["hold_endpoints"].as<bool>();
	system.useDummyPaths = vm["dummy_paths"].as<bool>();
    system.save_result = vm["save_result"].as<bool>();
    system.save_solver = vm["save_solver"].as<bool>();
    system.stop_at_traffic_jam = vm["stop_at_traffic_jam"].as<bool>();
    system.rotation_time = vm["rotation_time"].as<int>();
    system.queue_mechanism = vm["queue_mechanism"].as<bool>();
    system.port_number = vm["port_number"].as<int>();
    system.backup_solver = vm["backup_solver"].as<string>();
	if (vm.count("seed"))
		system.seed = vm["seed"].as<int>();
	else
		system.seed = (int)time(0);
	srand(system.seed);
}




MAPFSolver* set_solver(const BasicGraph& G, const boost::program_options::variables_map& vm)
{
	string solver_name = vm["single_agent_solver"].as<string>();
	SingleAgentSolver* path_planner;
	MAPFSolver* mapf_solver;
	if (solver_name == "ASTAR")
	{
		path_planner = new StateTimeAStar();
	}
	else if (solver_name == "SIPP")
	{
		path_planner = new SIPP();
	}
	else
	{
		cout << "Single-agent solver " << solver_name << "does not exist!" << endl;
		exit(-1);
	}

    // Set parameters for the path planner
    path_planner->rotation_time = vm["rotation_time"].as<int>();

	solver_name = vm["solver"].as<string>();
	if (solver_name == "ECBS")
	{
		ECBS* ecbs = new ECBS(G, *path_planner);
		ecbs->potential_function = vm["potential_function"].as<string>();
		ecbs->potential_threshold = vm["potential_threshold"].as<double>();
		ecbs->suboptimal_bound = vm["suboptimal_bound"].as<double>();
		mapf_solver = ecbs;
	}
	else if (solver_name == "PBS")
	{
		PBS* pbs = new PBS(G, *path_planner);
		pbs->lazyPriority = vm["lazyP"].as<bool>();
        bool prioritize_start = vm["prioritize_start"].as<bool>();
        if (vm["hold_endpoints"].as<bool>() or vm["dummy_paths"].as<bool>())
            prioritize_start = false;
		pbs->prioritize_start = prioritize_start;
		pbs->setRT(vm["CAT"].as<bool>(), vm["prioritize_start"].as<bool>());
		mapf_solver = pbs;
	}
	else if (solver_name == "WHCA")
	{
		mapf_solver = new WHCAStar(G, *path_planner);
	}
	else if (solver_name == "LRA")
	{
		mapf_solver = new LRAStar(G, *path_planner);
	}
    else if (solver_name == "PIBT")
    {
        mapf_solver = new PIBT(G, *path_planner);
    }
	else
	{
		cout << "Solver " << solver_name << "does not exist!" << endl;
		exit(-1);
	}

    // Set parameters for the MAPF solver
    if (vm.count("seed"))
        mapf_solver->seed = vm["seed"].as<int>();
    else
        mapf_solver->seed = (int)time(0);
    mapf_solver->gen = mt19937(mapf_solver->seed);

    // Number of iterations for the MAPF solver. Only applies to WHCA
    if (vm.count("n_iter_limit"))
        mapf_solver->n_iter_limit = vm["n_iter_limit"].as<int>();
    else
        mapf_solver->n_iter_limit = INT_MAX;

	if (vm["id"].as<bool>())
	{
		return new ID(G, *path_planner, *mapf_solver);
	}
	else
	{
		return mapf_solver;
	}
}


int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
    // clang-format off
	desc.add_options()
		("help", "produce help message")
		("scenario", po::value<std::string>()->required(), "scenario (SORTING, KIVA, ONLINE, BEE, MANUFACTURE, GREYORANGE)")
		("map,m", po::value<std::string>()->required(), "input map file")
		("task", po::value<std::string>()->default_value(""), "input task file")
        ("agents", po::value<std::string>()->default_value(""), "input agents file. Used by GREYORANGE scenario.")
		("output,o", po::value<std::string>()->default_value("output"), "output folder name")
		("agentNum,k", po::value<int>()->required(), "number of drives")
		("cutoffTime,t", po::value<int>()->default_value(60), "cutoff time (seconds) each time MAPF solver is called")
        ("OverallCutoffTime", po::value<int>()->default_value(INT_MAX / 2), "cutoff time (seconds) of the simulation")
		("seed,d", po::value<int>(), "random seed")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("solver", po::value<string>()->default_value("PBS"), "solver (LRA, PBS, WHCA, ECBS)")
        ("backup_solver", po::value<string>()->default_value("PIBT"), "backup solver (PIBT, LRA)")
		("id", po::value<bool>()->default_value(false), "independence detection")
		("single_agent_solver", po::value<string>()->default_value("SIPP"), "single-agent solver (ASTAR, SIPP)")
		("lazyP", po::value<bool>()->default_value(false), "use lazy priority")
		("simulation_time", po::value<int>()->default_value(5000), "run simulation")
		("simulation_window", po::value<int>()->default_value(5), "call the planner every simulation_window timesteps")
		("travel_time_window", po::value<int>()->default_value(0), "consider the traffic jams within the given window")
		("planning_window", po::value<int>()->default_value(INT_MAX / 2),
		        "the planner outputs plans with first planning_window timesteps collision-free")
		("potential_function", po::value<string>()->default_value("NONE"), "potential function (NONE, SOC, IC)")
		("potential_threshold", po::value<double>()->default_value(0), "potential threshold")
		("rotation", po::value<bool>()->default_value(false), "consider rotation")
        ("rotation_time", po::value<int>()->default_value(1), "number of timesteps to rotate. should be >= 1")
		("robust", po::value<int>()->default_value(0), "k-robust (for now, only work for PBS)")
		("CAT", po::value<bool>()->default_value(false), "use conflict-avoidance table")
		// ("PG", po::value<bool>()->default_value(false),
		//        "reuse the priority graph of the goal node of the previous search")
		("hold_endpoints", po::value<bool>()->default_value(false),
		        "Hold endpoints from Ma et al, AAMAS 2017")
		("dummy_paths", po::value<bool>()->default_value(false),
				"Find dummy paths from Liu et al, AAMAS 2019")
		("prioritize_start", po::value<bool>()->default_value(true), "Prioritize waiting at start locations")
		("suboptimal_bound", po::value<double>()->default_value(1), "Suboptimal bound for ECBS")
		("log", po::value<bool>()->default_value(false), "save the search trees (and the priority trees)")
        ("force_new_logdir", po::value<bool>()->default_value(true), "Force the program to create new hash tables even if it already exists.")
        ("save_result", po::value<bool>()->default_value(false), "Save result on disk. This could take too much save on disk in warehouse environment generation project")
        ("save_solver", po::value<bool>()->default_value(false), "Save solver result on disk. This could take too much save on disk in warehouse environment generation project")
        ("save_heuristics_table", po::value<bool>()->default_value(false), "Save heuristics table on disk. This could take too much save on disk in warehouse environment generation project")
        ("stop_at_traffic_jam", po::value<bool>()->default_value(true), "Whether to stop at traffic jam. Usually we want to stop for better performance.")
        ("left_w_weight", po::value<double>()->default_value(1.0), "weight of the workstations on the left border.")
        ("right_w_weight", po::value<double>()->default_value(1.0), "weight of the workstations on the right border.")
        ("n_station_types", po::value<int>()->default_value(3), "number of types of manufacture stations for manufacture scenario.")
        ("station_wait_times", po::value<std::vector<int>>()->multitoken()->default_value(vector<int>{1, 5, 10}), "wait time of each type of manufacture station.")
        ("queue_mechanism", po::value<bool>()->default_value(false), "Whether to use queue mechanism. This is only used for GreyOrange scenario.")
        ("port_number", po::value<int>()->default_value(8080), "port number for the server")
        ("grid_type", po::value<std::string>()->default_value("regular"), "Grid type (regular, one_bot_per_aisle)")
		;
    // clang-format on
	clock_t start_time = clock();
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);

    // check params
    if (vm["hold_endpoints"].as<bool>() or vm["dummy_paths"].as<bool>())
    {
        if (vm["hold_endpoints"].as<bool>() and vm["dummy_paths"].as<bool>())
        {
            std::cerr << "Hold endpoints and dummy paths cannot be used simultaneously" << endl;
            exit(-1);
        }
        if (vm["simulation_window"].as<int>() != 1)
        {
            std::cerr << "Hold endpoints and dummy paths can only work when the simulation window is 1" << endl;
            exit(-1);
        }
        if (vm["planning_window"].as<int>() < INT_MAX / 2)
        {
            std::cerr << "Hold endpoints and dummy paths cannot work with planning windows" << endl;
            exit(-1);
        }
    }

    // make dictionary
    bool force_new_logdir = vm["force_new_logdir"].as<bool>();
	boost::filesystem::path dir(vm["output"].as<std::string>() +"/");

    // Remove previous dir if necessary.
    if (boost::filesystem::exists(dir) && force_new_logdir)
    {
        boost::filesystem::remove_all(dir);
    }

    // Make a new log dir only if we need to log something
	if (vm["log"].as<bool>() ||
        vm["save_heuristics_table"].as<bool>() ||
        vm["save_result"].as<bool>() ||
        vm["save_solver"].as<bool>())
	{
        boost::filesystem::create_directories(dir);
		boost::filesystem::path dir1(vm["output"].as<std::string>() + "/goal_nodes/");
		boost::filesystem::path dir2(vm["output"].as<std::string>() + "/search_trees/");
		boost::filesystem::create_directories(dir1);
		boost::filesystem::create_directories(dir2);
	}

    // Set up logger
    auto console_logger = spdlog::default_logger()->clone("Planner");
    spdlog::set_default_logger(console_logger);

    // Grid type
    std::string grid_type = vm["grid_type"].as<std::string>();

	if (vm["scenario"].as<string>() == "KIVA")
	{
		KivaGrid G;
		G.screen = vm["screen"].as<int>();
        G.hold_endpoints = vm["hold_endpoints"].as<bool>();
	    G.useDummyPaths = vm["dummy_paths"].as<bool>();
        G._save_heuristics_table = vm["save_heuristics_table"].as<bool>();
        G.rotation_time = vm["rotation_time"].as<int>();
        std::ifstream i(vm["map"].as<std::string>());
        json map_json;
        i >> map_json;
		if (!G.load_map_from_jsonstr(
                map_json.dump(4),
                vm["left_w_weight"].as<double>(),
                vm["right_w_weight"].as<double>()))
        {
            return -1;
        }
		MAPFSolver* solver = set_solver(G, vm);
        solver->consider_task_wait = true;
		KivaSystem system(G, *solver);
		set_parameters(system, vm);
        system.start_time = start_time;
		G.preprocessing(system.consider_rotation, "map");
		system.simulate(vm["simulation_time"].as<int>());
        double runtime = (double)(clock() - start_time)/ CLOCKS_PER_SEC;
		cout << "Overall runtime: " << runtime << " seconds." << endl;
		return 0;
	}
    else if (vm["scenario"].as<string>() == "SMART")
	{
		SMARTGrid G;
		G.screen = vm["screen"].as<int>();
        G.hold_endpoints = vm["hold_endpoints"].as<bool>();
	    G.useDummyPaths = vm["dummy_paths"].as<bool>();
        G._save_heuristics_table = vm["save_heuristics_table"].as<bool>();
        G.rotation_time = vm["rotation_time"].as<int>();
        // Grid type
        if (!convert_G_type.count(grid_type))
        {
            spdlog::error("Grid type {} does not exist!", grid_type);
            exit(-1);
        }
        G.set_grid_type(convert_G_type.at(grid_type));
        std::ifstream i(vm["map"].as<std::string>());
        json map_json;
        i >> map_json;
		if (!G.load_map_from_jsonstr(
                map_json.dump(4),
                vm["left_w_weight"].as<double>(),
                vm["right_w_weight"].as<double>()))
        {
            return -1;
        }
		MAPFSolver* solver = set_solver(G, vm);
        solver->consider_task_wait = true;
		SMARTSystem system(G, *solver, vm["task"].as<string>());
		set_parameters(system, vm);
        system.start_time = start_time;
		G.preprocessing(system.consider_rotation, "map");
		system.simulate(vm["simulation_time"].as<int>());
        double runtime = (double)(clock() - start_time)/ CLOCKS_PER_SEC;
		cout << "Overall runtime: " << runtime << " seconds." << endl;
		return 0;
	}
	else
	{
		cout << "Scenario " << vm["scenario"].as<string>() << "does not exist!" << endl;
		return -1;
	}
}

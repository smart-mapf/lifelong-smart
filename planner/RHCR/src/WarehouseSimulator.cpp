#include "WarehouseSimulator.h"
#include "helper.h"

PYBIND11_MODULE(WarehouseSimulator, m){
    py::class_<WarehouseSimulator>(m, "WarehouseSimulator")
        .def(py::init<py::kwargs>())
        .def("warmup", &WarehouseSimulator::warmup)
        .def("update_gg_and_step", &WarehouseSimulator::update_gg_and_step);
}

std::string WarehouseSimulator::warmup(){
    this->G->preprocessing(this->system->consider_rotation, this->output);
    // std::cout << this->G->heuristics.size() << std::endl;
    // std::cout << this->system->G.heuristics.size() << std::endl;
    // exit(1);
    json result = this->system->warmup(this->warmup_time);
    // std::cout << "solver name: " <<this->system->solver.get_name()<<std::endl;
    return result.dump(4);
}

std::string WarehouseSimulator::update_gg_and_step(bool optimize_wait, std::vector<double> weights){
    this->G->reset_weights(this->system->consider_rotation, this->output, optimize_wait, weights);
    json result = this->system->update_gg_and_step(this->update_gg_interval);
    return result.dump(4);
}

// void set_parameters(BasicSystem& system, const py::kwargs& kwargs)
// {
// 	system.outfile = kwargs["output"].cast<std::string>();
// 	system.screen = kwargs["screen"].cast<int>();
// 	system.log = kwargs["log"].cast<bool>();
// 	system.num_of_drives = kwargs["agentNum"].cast<int>();
// 	system.time_limit = kwargs["cutoffTime"].cast<int>();
//     system.overall_time_limit = kwargs["OverallCutoffTime"].cast<int>();
// 	system.simulation_window = kwargs["simulation_window"].cast<int>();
// 	system.planning_window = kwargs["planning_window"].cast<int>();
// 	system.travel_time_window = kwargs["travel_time_window"].cast<int>();
// 	system.consider_rotation = kwargs["rotation"].cast<bool>();
// 	system.k_robust = kwargs["robust"].cast<int>();
// 	system.hold_endpoints = kwargs["hold_endpoints"].cast<bool>();
// 	system.useDummyPaths = kwargs["dummy_paths"].cast<bool>();
//     system.save_result = kwargs["save_result"].cast<bool>();
//     system.save_solver = kwargs["save_solver"].cast<bool>();
//     system.stop_at_traffic_jam = kwargs["stop_at_traffic_jam"].cast<bool>();
// 	if (kwargs.contains("seed"))
// 		system.seed = kwargs["seed"].cast<int>();
// 	else
// 		system.seed = (int)time(0);
// 	srand(system.seed);

// 	// if (kwargs.contains("dist_sigma")){
// 	// 	dist_params.sigma = kwargs["dist_sigma"].cast<double>();
// 	// }
// 	// if (kwargs.contains("dist_K")){
// 	// 	dist_params.K = kwargs["dist_K"].cast<int>();
// 	// }
// }


// MAPFSolver* set_solver(const BasicGraph& G, const py::kwargs& kwargs)
// {
// 	string solver_name = kwargs["single_agent_solver"].cast<string>();
// 	SingleAgentSolver* path_planner;
// 	MAPFSolver* mapf_solver;
// 	if (solver_name == "ASTAR")
// 	{
// 		path_planner = new StateTimeAStar();
// 	}
// 	else if (solver_name == "SIPP")
// 	{
// 		path_planner = new SIPP();
// 	}
// 	else
// 	{
// 		cout << "Single-agent solver " << solver_name << "does not exist!" << endl;
// 		exit(-1);
// 	}

// 	solver_name = kwargs["solver"].cast<string>();
//     std::cout << "solver_name = "<<solver_name <<std::endl;
// 	if (solver_name == "ECBS")
// 	{
// 		ECBS* ecbs = new ECBS(G, *path_planner);
// 		ecbs->potential_function = kwargs["potential_function"].cast<string>();
// 		ecbs->potential_threshold = kwargs["potential_threshold"].cast<double>();
// 		ecbs->suboptimal_bound = kwargs["suboptimal_bound"].cast<double>();
// 		mapf_solver = ecbs;
// 	}
// 	else if (solver_name == "PBS")
// 	{
// 		PBS* pbs = new PBS(G, *path_planner);
// 		pbs->lazyPriority = kwargs["lazyP"].cast<bool>();
//         bool prioritize_start = kwargs["prioritize_start"].cast<bool>();
//         if (kwargs["hold_endpoints"].cast<bool>() ||
//             kwargs["dummy_paths"].cast<bool>())
//             prioritize_start = false;
// 		pbs->prioritize_start = prioritize_start;
// 		pbs->prioritize_start = kwargs["prioritize_start"].cast<bool>();
// 		pbs->setRT(kwargs["CAT"].cast<bool>(), kwargs["prioritize_start"].cast<bool>());
// 		mapf_solver = pbs;
// 	}
// 	else if (solver_name == "WHCA")
// 	{
// 		mapf_solver = new WHCAStar(G, *path_planner);
// 	}
// 	else if (solver_name == "LRA")
// 	{
// 		mapf_solver = new LRAStar(G, *path_planner);
// 	}
// 	else
// 	{
// 		cout << "Solver " << solver_name << "does not exist!" << endl;
// 		exit(-1);
// 	}

// 	if (kwargs["id"].cast<bool>())
// 	{
// 		return new ID(G, *path_planner, *mapf_solver);
// 	}
// 	else
// 	{
// 		return mapf_solver;
// 	}
// }

WarehouseSimulator::WarehouseSimulator(py::kwargs kwargs){
    if (!kwargs.contains("planning_window")){
        kwargs["planning_window"] = INT_MAX / 2;
    }

    if (!kwargs.contains("left_w_weight")){
        kwargs["left_w_weight"] = 1.0;
    }

    if (!kwargs.contains("right_w_weight")){
        kwargs["right_w_weight"] = 1.0;
    }

    if (!kwargs.contains("OverallCutoffTime")){
        kwargs["OverallCutoffTime"] = INT_MAX / 2;
    }

    namespace po = boost::program_options;
    clock_t start_time = clock();
	json result;
	result["finish"] = false; // status code to false by default

    // check params
    if (kwargs["hold_endpoints"].cast<bool>() or kwargs["dummy_paths"].cast<bool>()){
        if (kwargs["hold_endpoints"].cast<bool>() and kwargs["dummy_paths"].cast<bool>()){
            std::cerr << "Hold endpoints and dummy paths cannot be used simultaneously" << endl;
            exit(-1);
        }
        if (kwargs["simulation_window"].cast<int>() != 1){
            std::cerr << "Hold endpoints and dummy paths can only work when the simulation window is 1" << endl;
            exit(-1);
        }
        if (kwargs["planning_window"].cast<int>() < INT_MAX / 2){
            std::cerr << "Hold endpoints and dummy paths cannot work with planning windows" << endl;
            exit(-1);
        }
    }

    // make dictionary
    bool force_new_logdir = kwargs["force_new_logdir"].cast<bool>();
	boost::filesystem::path dir(kwargs["output"].cast<std::string>() +"/");

    // Remove previous dir is necessary.
    if (boost::filesystem::exists(dir) && force_new_logdir){
        boost::filesystem::remove_all(dir);
    }

    if (kwargs["log"].cast<bool>() ||
        kwargs["save_heuristics_table"].cast<bool>() ||
        kwargs["save_result"].cast<bool>() ||
        kwargs["save_solver"].cast<bool>()){
        boost::filesystem::create_directories(dir);
		boost::filesystem::path dir1(kwargs["output"].cast<std::string>() + "/goal_nodes/");
		boost::filesystem::path dir2(kwargs["output"].cast<std::string>() + "/search_trees/");
		boost::filesystem::create_directories(dir1);
		boost::filesystem::create_directories(dir2);
	}

    std::string scenario = kwargs["scenario"].cast<std::string>();
    if (scenario == "KIVA"){
        this->G = new KivaGrid();
        this->G->screen = kwargs["screen"].cast<int>();
        this->G->hold_endpoints = kwargs["hold_endpoints"].cast<bool>();
        this->G->useDummyPaths = kwargs["dummy_paths"].cast<bool>();
        this->G->_save_heuristics_table = kwargs["save_heuristics_table"].cast<bool>();
        if (!this->G->load_map_from_jsonstr(
                kwargs["map"].cast<std::string>(),
                kwargs["left_w_weight"].cast<double>(),
                kwargs["right_w_weight"].cast<double>())){
            std::cout << "map loading error" <<std::endl;
            exit(-1);
        }

        this->solver = helper::set_solver(*this->G, kwargs);
        // this->system = new KivaSystem(this->G, *this->solver);
        this->system = new KivaSystem(
            *(dynamic_cast<KivaGrid*>(this->G)), *this->solver);
        helper::set_parameters(*this->system, kwargs);
        this->system->start_time = start_time;

        this->output = kwargs["output"].cast<std::string>();

        this->warmup_time = kwargs["warmup_time"].cast<int>();
        this->update_gg_interval = kwargs["update_gg_interval"].cast<int>();
        this->system->set_total_sim_time(kwargs["simulation_time"].cast<int>(), this->warmup_time);

        // if (kwargs.contains("task_dist_update_interval")){
        //     this->system->task_dist_update_interval = kwargs["task_dist_update_interval"].cast<int>();
        // }
        // if (kwargs.contains("task_dist_type")){
        //     this->system->task_dist_type = kwargs["task_dist_type"].cast<std::string>();
        // }
    }
    else if (scenario == "SORTING"){
        this->G = new SortingGrid();
        this->G->screen = kwargs["screen"].cast<int>();
        this->G->hold_endpoints = kwargs["hold_endpoints"].cast<bool>();
        this->G->useDummyPaths = kwargs["dummy_paths"].cast<bool>();
        this->G->_save_heuristics_table = kwargs["save_heuristics_table"].cast<bool>();
        if (!this->G->load_map_from_jsonstr(
                kwargs["map"].cast<std::string>(),
                kwargs["left_w_weight"].cast<double>(),
                kwargs["right_w_weight"].cast<double>())){
            std::cout << "map loading error" <<std::endl;
            exit(-1);
        }

        this->solver = helper::set_solver(*this->G, kwargs);

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

        this->system = new SortingSystem(
            *(dynamic_cast<SortingGrid*>(this->G)), *this->solver,
            chute_mapping_int, package_mode, packages, package_dist_weight,
            task_assignment_cost, task_assignment_params);
        helper::set_parameters(*this->system, kwargs);
        this->system->start_time = start_time;

        this->output = kwargs["output"].cast<std::string>();

        this->warmup_time = kwargs["warmup_time"].cast<int>();
        this->update_gg_interval = kwargs["update_gg_interval"].cast<int>();
        this->system->set_total_sim_time(kwargs["simulation_time"].cast<int>(), this->warmup_time);
    }
    else{
        cout << "Scenario " << scenario << "does not exist!" << endl;
        exit(-1);
    }

    // G.preprocessing(system.consider_rotation,
    //                 kwargs["output"].cast<std::string>());
    // result = system.simulate(kwargs["simulation_time"].cast<int>());
    // result["finish"] = true; // Change status code
    // double runtime = (double)(clock() - start_time)/ CLOCKS_PER_SEC;
    // cout << "Overall runtime: " << runtime << " seconds." << endl;
    // result["cpu_runtime"] = runtime;
}

WarehouseSimulator::~WarehouseSimulator(){
    delete this->G;
    delete this->system;
    delete this->solver;
}
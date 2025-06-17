#include "helper.h"
#include "ID.h"
#include <numeric>
#include <algorithm>
#include <math.h>
#include <tuple>

#define R (8417508174513LL)
#define X (165131LL)

#define MOD(x) ((x % R + R) % R)

namespace helper
{
    std::tuple<double, double> mean_std(std::vector<double> v)
    {
        if (v.size() == 0)
            return std::make_tuple(0, 0);
        double sum = helper::sum(v);
        double mean = sum / v.size();

        std::vector<double> diff(v.size());
        std::transform(v.begin(), v.end(), diff.begin(),
                       [mean](double x)
                       { return x - mean; });

        double sq_sum = std::inner_product(diff.begin(), diff.end(),
                                           diff.begin(), 0.0);
        double sigma = sqrt(sq_sum / v.size());
        return std::make_tuple(mean, sigma);
    }

    void divide(std::vector<double> &v, double factor)
    {
        for (int i = 0; i < v.size(); i += 1)
        {
            v[i] /= factor;
        }
    }

    double sum(std::vector<double> v)
    {
        double sum = std::accumulate(v.begin(), v.end(), 0.0);
        return sum;
    }

    // **** Functions to get the longest common sub-path ****
    // Reference: https://leetcode.com/problems/longest-common-subpath/solutions/1319639/c-easy-solution-using-rolling-hash-with-explanation/

    // Return a tuple of <valid, start_idx, end_idx>, where valid indicates
    // whether there is a path of length L shared by all agents, and start/end
    // idx is the start and end index of the path in path[0].
    std::tuple<bool, int, int> is_valid(int L, std::vector<Path> &paths)
    {
        long long hash = 1;
        for (int i = 0; i < L - 1; ++i)
            // calculates hash of first path of length L in paths[0]
            hash = MOD(hash * X);

        // Map from hash value of a path to the number of the path.
        std::map<long long, int> mark;
        // Map from hash value to the start and end index of the path in path[0]
        std::map<long long, std::tuple<int, int>> marked_paths;
        auto &p0 = paths[0];
        long long v = 0;
        for (int i = 0; i < p0.size(); ++i)
        {
            v = MOD(v * X + p0[i].location); // calculates running hash
            if (i >= L - 1)
            {
                mark[v] = 1; // when L length subpath is found, hash it
                marked_paths[v] = make_tuple(i - L + 1, i);

                // here the value can be negative
                // that's why MOD is defined that way
                // subtract the previous part of hash to include the next part
                v = MOD(v - p0[i - L + 1].location * hash);
            }
        }

        for (int p = 1; p < paths.size(); ++p)
        {
            v = 0;
            // traverse all paths to check if any of the hash value is present
            auto &pth = paths[p];

            for (int i = 0; i < pth.size(); ++i)
            {
                v = MOD(v * X + pth[i].location);
                if (i >= L - 1)
                {
                    // only the hash which is present in all previous paths is
                    // increased
                    if (mark.count(v) > 0 && mark[v] == p)
                    {
                        mark[v] += 1;
                    }
                    // subtract the previous part of hash to include the next
                    // part
                    v = MOD(v - pth[i - L + 1].location * hash);
                }
            }
        }

        for (auto it : mark)
        {
            // a hash that is present in all paths
            if (it.second == paths.size())
            {
                int start_idx, end_idx;
                std::tie(start_idx, end_idx) = marked_paths[it.first];
                return make_tuple(true, start_idx, end_idx);
            }
        }
        return make_tuple(false, -1, -1);
    }

    static bool compare(Path a, Path b)
    {
        return (a.size() < b.size());
    }

    vector<int> longest_common_subpath(vector<Path> &paths, int simulation_time)
    {
        int ans_start, ans_end;
        bool has_valid = false;

        vector<Path> real_paths(paths.size());

        // Remove duplicates
        for (int i = 0; i < paths.size(); i++)
        {
            Path real_path;
            int t = 0;
            while (t < paths[i].size() && t < simulation_time)
            {
                State curr = paths[i][t];
                real_path.emplace_back(curr);
                // Skip waits
                while (t < paths[i].size() &&
                       curr.location == paths[i][t].location)
                    t += 1;
            }
            real_paths[i] = real_path;
        }

        // cout << "Real paths" << endl;
        // for (int i = 0; i < real_paths.size(); i++)
        // {
        //     cout << real_paths[i].size() << endl;
        // }

        // sort paths in increasing order of size
        sort(real_paths.begin(), real_paths.end(), compare);

        int l = 0, r = real_paths[0].size() + 1;
        while (r - l > 1)
        {
            // choose a length for subpath in smallest length path
            int mid = (r + l) / 2;
            bool valid;
            int start_idx, end_idx;
            std::tie(valid, start_idx, end_idx) = is_valid(mid, real_paths);
            if (valid)
            {
                // m will be an ans, but we want the largest subpath
                l = mid;
                ans_start = start_idx;
                ans_end = end_idx;
                has_valid = true;
            }
            else
                r = mid;
        }

        // We only care about the locations.
        vector<int> ans;
        if (has_valid)
        {
            // Construct the longest common subpath
            for (int i = ans_start; i <= ans_end; i++)
                ans.emplace_back(real_paths[0][i].location);
        }
        return ans;
    }

    // **** End Functions to get the longest common sub-path ****

    void set_parameters(BasicSystem &system, const py::kwargs &kwargs)
    {
        system.outfile = kwargs["output"].cast<std::string>();
        system.screen = kwargs["screen"].cast<int>();
        system.log = kwargs["log"].cast<bool>();
        system.num_of_drives = kwargs["agentNum"].cast<int>();
        system.time_limit = kwargs["cutoffTime"].cast<int>();
        system.overall_time_limit = kwargs["OverallCutoffTime"].cast<int>();
        system.simulation_window = kwargs["simulation_window"].cast<int>();
        system.planning_window = kwargs["planning_window"].cast<int>();
        system.travel_time_window = kwargs["travel_time_window"].cast<int>();
        system.consider_rotation = kwargs["rotation"].cast<bool>();
        system.k_robust = kwargs["robust"].cast<int>();
        system.hold_endpoints = kwargs["hold_endpoints"].cast<bool>();
        system.useDummyPaths = kwargs["dummy_paths"].cast<bool>();
        system.save_result = kwargs["save_result"].cast<bool>();
        system.save_solver = kwargs["save_solver"].cast<bool>();
        system.stop_at_traffic_jam = kwargs["stop_at_traffic_jam"].cast<bool>();
        system.rotation_time = kwargs["rotation_time"].cast<int>();
        if (kwargs.contains("seed"))
            system.seed = kwargs["seed"].cast<int>();
        else
            system.seed = (int)time(0);
        srand(system.seed);

        if (kwargs.contains("queue_mechanism"))
            system.queue_mechanism = kwargs["queue_mechanism"].cast<bool>();
        else
            system.queue_mechanism = false;

        // if (kwargs.contains("task_dist_update_interval")){
        //     system.task_dist_update_interval = kwargs["task_dist_update_interval"].cast<int>();
        // }
        // if (kwargs.contains("task_dist_type")){
        //     system.task_dist_type = kwargs["task_dist_type"].cast<std::string>();
        // }

        // if (kwargs.contains("dist_sigma")){
        // 	dist_params.sigma = kwargs["dist_sigma"].cast<double>();
        // }
        // if (kwargs.contains("dist_K")){
        // 	dist_params.K = kwargs["dist_K"].cast<int>();
        // }
    }

    MAPFSolver *set_solver(const BasicGraph &G, const py::kwargs &kwargs)
    {
        string solver_name = kwargs["single_agent_solver"].cast<string>();
        SingleAgentSolver *path_planner;
        MAPFSolver *mapf_solver;
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
        path_planner->rotation_time = kwargs["rotation_time"].cast<int>();

        solver_name = kwargs["solver"].cast<string>();
        if (solver_name == "ECBS")
        {
            ECBS *ecbs = new ECBS(G, *path_planner);
            ecbs->potential_function = kwargs["potential_function"].cast<string>();
            ecbs->potential_threshold = kwargs["potential_threshold"].cast<double>();
            ecbs->suboptimal_bound = kwargs["suboptimal_bound"].cast<double>();
            mapf_solver = ecbs;
        }
        else if (solver_name == "PBS")
        {
            PBS *pbs = new PBS(G, *path_planner);
            pbs->lazyPriority = kwargs["lazyP"].cast<bool>();
            bool prioritize_start = kwargs["prioritize_start"].cast<bool>();
            if (kwargs["hold_endpoints"].cast<bool>() ||
                kwargs["dummy_paths"].cast<bool>())
                prioritize_start = false;
            pbs->prioritize_start = prioritize_start;
            pbs->prioritize_start = kwargs["prioritize_start"].cast<bool>();
            pbs->setRT(kwargs["CAT"].cast<bool>(), kwargs["prioritize_start"].cast<bool>());
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
        else
        {
            cout << "Solver " << solver_name << "does not exist!" << endl;
            exit(-1);
        }

        // Set parameters for the MAPF solver
        if (kwargs.contains("seed"))
            mapf_solver->seed = kwargs["seed"].cast<int>();
        else
            mapf_solver->seed = (int)time(0);
        mapf_solver->gen = mt19937(mapf_solver->seed);

        // Number of iterations for the MAPF solver. Only applies to WHCA
        if (kwargs.contains("n_iter_limit"))
            mapf_solver->n_iter_limit = kwargs["n_iter_limit"].cast<int>();
        else
            mapf_solver->n_iter_limit = INT_MAX;

        if (kwargs["id"].cast<bool>())
        {
            return new ID(G, *path_planner, *mapf_solver);
        }
        else
        {
            return mapf_solver;
        }
    }


    std::tuple<string, vector<int>, vector<double>> setup_packages(
        const py::kwargs& kwargs)
    {
        // For packages, the program accepts a list of explicit "packages" or a
        // distribution of packages on each destination
        std::string package_mode = kwargs["package_mode"].cast<std::string>();
        vector<int> packages;
        vector<double> package_dist_weight;
        cout << "creating package dist" << endl;
        if (package_mode == "explicit")
        {
            if (!kwargs.contains("packages"))
            {
                std::cerr << "packages must be provided for sortation system" << endl;
                exit(1);
            }
            nlohmann::json packages_json = nlohmann::json::parse(
                kwargs["packages"].cast<std::string>());
            packages = packages_json.get<vector<int>>();
        }
        else if (package_mode == "dist")
        {
            if (!kwargs.contains("package_dist_weight"))
            {
                std::cerr << "package_dist_weight must be provided for sortation system" << endl;
                exit(1);
            }
            nlohmann::json package_dist_weight_json = nlohmann::json::parse(
                kwargs["package_dist_weight"].cast<std::string>());
            package_dist_weight = package_dist_weight_json.get<vector<double>>();
        }
        else
        {
            std::cerr << "packages or package_dist_weight must be provided for sortation system" << endl;
            exit(1);
        }
        return make_tuple(package_mode, packages, package_dist_weight);
    }

    map<int, vector<int>> setup_chute_mapping(const py::kwargs& kwargs)
    {
        cout << "Loading chute mapping" << endl;
        nlohmann::json chute_mapping_json = nlohmann::json::parse(
            kwargs["chute_mapping"].cast<std::string>());
        // cout << chute_mapping_json << endl;
        map<string, vector<int>> chute_mapping = chute_mapping_json.get<map<string, vector<int>>>();

        // Transform map<string, vector<int>> to a map<int, vector<int>>
        map<int, vector<int>> chute_mapping_int;
        for (auto const &pair : chute_mapping)
        {
            chute_mapping_int[stoi(pair.first)] = pair.second;
        }

        // // Print content of packages
        // for (int i = 0; i < packages.size(); i++)
        // {
        //     std::cout << packages[i] << " ";
        // }
        // cout << endl;
        // Print content of chute_mapping
        // for (auto const &pair : chute_mapping_int)
        // {
        //     std::cout << "{" << pair.first << ": ";
        //     for (int i = 0; i < pair.second.size(); i++)
        //     {
        //         std::cout << pair.second[i] << " ";
        //     }
        //     std::cout << "}" << std::endl;
        // }
        return chute_mapping_int;
    }

    std::tuple<string, vector<double>> setup_task_assign(
        const py::kwargs& kwargs)
    {
        // Task assignment policy
        string task_assignment_cost=kwargs["task_assignment_cost"].cast<std::string>();
        // If the task assignment policy is of an optimization-based policy,
        // read the parameters
        vector<double> task_assignment_params;
        if (task_assignment_cost == "opt_quadratic_f")
        {
            nlohmann::json task_assignment_params_json = nlohmann::json::parse(
                kwargs["task_assignment_params"].cast<std::string>());
            task_assignment_params = task_assignment_params_json.get<vector<double>>();
        }
        return make_tuple(task_assignment_cost, task_assignment_params);
    }
}

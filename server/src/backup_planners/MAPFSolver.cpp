#include "backup_planners/MAPFSolver.h"

#include <ctime>
#include <iostream>
// #include "backup_planners/PathTable.h"

MAPFSolver::MAPFSolver(const BasicGraph& G, SingleAgentSolver& path_planner)
    : solution_found(false),
      solution_cost(-2),
      avg_path_length(-1),
      G(G),
      path_planner(path_planner),
      initial_rt(G),
      rt(G) {
}

MAPFSolver::~MAPFSolver() {
}

bool MAPFSolver::validate_solution() {
    // Check whether the paths are feasible.
    double soc = 0;
    for (int a1 = 0; a1 < num_of_agents; a1++) {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++) {
            size_t min_path_length = this->simulation_window;
            for (size_t timestep = 0; timestep < min_path_length; timestep++) {
                int loc1 = this->solution[a1][timestep].location;
                int loc2 = this->solution[a2][timestep].location;
                if (loc1 == loc2) {
                    cout << "Agents " << a1 << " and " << a2 << " collides at "
                         << loc1 << " at timestep " << timestep << endl;
                    return false;
                } else if (timestep < min_path_length - 1 &&
                           loc1 == this->solution[a2][timestep + 1].location &&
                           loc2 == this->solution[a1][timestep + 1].location) {
                    cout << "Agents " << a1 << " and " << a2 << " collides at ("
                         << loc1 << "-->" << loc2 << ") at timestep "
                         << timestep << endl;
                    return false;
                }
            }

            // Don't need target conflict as the agents will disappear at goal.
            // if (this->solution[a1]->size() !=
            // this->solution[a2]->size())
            // {
            // 	int a1_ = this->solution[a1]->size() <
            // this->solution[a2]->size() ? a1 : a2; 	int a2_ =
            // this->solution[a1]->size() <
            // this->solution[a2]->size() ? a2 : a1; 	int loc1 =
            // this->solution[a1_]->back().location; 	for (size_t
            // timestep = min_path_length; timestep <
            // this->solution[a2_]->size(); timestep++)
            // 	{
            // 		int loc2 =
            // this->solution[a2_][timestep).location; 		if (loc1
            // == loc2)
            // 		{
            // 			cout << "Agents " << a1 << " and " << a2 << "
            // collides at
            // "
            // << loc1 << " at timestep " << timestep << endl;
            // return false; // It's at least a semi conflict
            // 		}
            // 	}
            // }
        }
    }
    return true;
}

void MAPFSolver::print_solution() const {
    for (int i = 0; i < num_of_agents; i++) {
        cout << "Agent " << i << ":\t";
        for (const auto& loc : solution[i]) {
            cout << loc.location;
            if (loc.is_tasking_wait)
                cout << "(wait)";
            cout << "->";
        }
        cout << endl;
    }
}

vector<vector<tuple<int, int, double, int>>> MAPFSolver::convert_path_to_smart(
    const vector<vector<Task>>& goal_locations) {
    vector<vector<tuple<int, int, double, int>>> new_mapf_plan;
    new_mapf_plan.resize(this->num_of_agents);
    if (screen > 0) {
        // cout << "######################################" << endl;
        // printf("Num of agents: %d\n", this->num_of_agents);
        spdlog::info("Backup Plan Result:");
    }

    vector<Path> raw_new_path = this->solution;

    for (int i = 0; i < this->num_of_agents; i++) {
        if (screen > 0) {
            cout << "Agent " << i << ": ";
        }
        // The first simulation_window steps of the path are collision free
        int goal_id = 0;
        for (int t = 0; t < this->simulation_window; t++) {
            // Infer task id
            State s = raw_new_path[i][t];
            int task_id = -1;
            Task curr_goal = goal_locations[i][goal_id];
            if (goal_locations[i].size() > goal_id &&
                curr_goal.location == s.location &&
                (curr_goal.orientation == -1 ||
                 curr_goal.orientation >= 0 &&
                     curr_goal.orientation == s.orientation)) {
                task_id = curr_goal.id;
                goal_id += 1;
            }
            new_mapf_plan[i].emplace_back(this->G.getRowCoordinate(s.location),
                                          this->G.getColCoordinate(s.location),
                                          static_cast<double>(t), task_id);
            if (screen > 0) {
                cout << "(" << "t=" << s.timestep << ","
                     << this->G.getRowCoordinate(s.location) << ","
                     << this->G.getColCoordinate(s.location) << ","
                     << s.orientation << "," << task_id << ")->";
                if (task_id >= 0) {
                    cout << "End of task " << task_id << " at step " << t
                         << "->";
                }
            }
        }
        if (screen > 0) {
            cout << endl;
        }
    }
    return new_mapf_plan;
}

// void MAPFSolver::print_mapf_instance(vector<State> starts_,
//                                      vector<vector<Task>> goals_) const {
//     for (int i = 0; i < starts_.size(); i++) {
//         cout << "Agent " << i << ": ";
//         int start_x = G.getRowCoordinate(starts_[i].location);
//         int start_y = G.getColCoordinate(starts_[i].location);
//         cout << "(" << start_x << ", " << start_y << ", "
//              << starts_[i].orientation << ", t = " << starts_[i].timestep
//              << ") => ";
//         for (const auto& goal : goals_[i]) {
//             int goal_x = G.getRowCoordinate(goal.location);
//             int goal_y = G.getColCoordinate(goal.location);
//             int wait_t = goal.task_wait_time;
//             cout << "(" << goal_x << ", " << goal_y << ", "
//                  << this->G.types[goal.location] << ", "
//                  << "wait: " << wait_t << ") -> ";
//         }
//         cout << endl;
//     }
// }

bool MAPFSolver::congested() const {
    // Count the number of agents that are not making progress in the current
    int wait_agents = 0;
    for (const auto& path : this->solution) {
        int t = 0;
        while (t < simulation_window && path[0].location == path[t].location &&
               path[0].orientation == path[t].orientation &&
               !path[0].is_rotating)
            t++;
        if (t == simulation_window)
            wait_agents++;
    }
    // more than half of drives didn't make progress
    return wait_agents > num_of_agents / 2;
}

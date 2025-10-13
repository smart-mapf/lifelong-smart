#include "task_assigners/OneGoalTaskAssigner.h"

OneGoalTaskAssigner::OneGoalTaskAssigner(const SMARTGrid& G, int screen,
                                         int num_of_agents, int seed,
                                         string task_file)
    : BasicTaskAssigner(G, screen, num_of_agents, seed, task_file) {
    // Each agent has only one goal at a time
    for (int i = 0; i < num_of_agents; i++) {
        this->goal_locations[i].resize(1);
        this->goal_locations[i][0] = Task();
        this->goal_locations[i][0].id = -1;  // no goal assigned
    }
}

void OneGoalTaskAssigner::updateStartsAndGoals(
    vector<tuple<double, double, int>>& start_locs,
    set<int> finished_tasks_id) {
    // Set new starts
    for (int i = 0; i < this->num_of_agents; i++) {
        // Obtain the starts
        int row = static_cast<int>(std::get<0>(start_locs[i]));
        int col = static_cast<int>(std::get<1>(start_locs[i]));
        int ori = std::get<2>(start_locs[i]);
        this->starts[i] = State(this->G.getCellId(row, col), 0,
                                this->G.ORI_SMART_TO_RHCR.at(ori));
    }

    // Remove finished tasks from goal locations
    for (int i = 0; i < num_of_agents; i++) {
        if (finished_tasks_id.find(goal_locations[i].begin()->id) !=
            finished_tasks_id.end()) {
            goal_locations[i].begin()->id = -1;  // reset goal id
        }
    }

    // Generate new goals
    for (int i = 0; i < num_of_agents; i++) {
        // generate a goal for the agent if it does not have one
        if (goal_locations[i].begin()->id == -1) {
            if (screen > 1)
                spdlog::info("Agent {} has no goal, generating a new one.", i);
            int curr_goal = goal_locations[i].begin()->location;
            int next_goal = this->genGoal(curr_goal, starts[i].location, i);
            goal_locations[i][0] =
                Task(next_goal, -1, 0, 0, i, -1, false, false, this->task_id);
            this->task_id++;
        }
    }
}

// Return the goal loc and if the goal is a backup goal.
int OneGoalTaskAssigner::genGoal(int curr_goal, int start_loc, int agent_id) {
    int next_goal;
    if (screen > 1) {
        spdlog::info("Generating a goal location from start location {}.",
                     start_loc);
    }

    // Warehouse tasks alternate between workstations and endpoints
    if (this->G.workstations.size() > 0 && this->G.endpoints.size() > 0) {
        if (screen > 1)
            spdlog::info("Generating goal for warehouse tasks.");
        // This is the first goal, determine the next goal based on current
        // location.
        if (curr_goal == -1) {
            if (screen > 1)
                spdlog::info("No current goal, generating the first goal.");
            if (this->G.types[start_loc] == "Workstation") {
                next_goal = sample_endpiont();
            } else if (this->G.types[start_loc] == "Endpoint") {
                next_goal = sample_workstation();
            } else {
                next_goal = this->sample_task_location();
            }
        }
        // Subsequent goals alternate between workstations and endpoints
        else {
            // If the current goal is a workstation, the next goal is an
            // endpoint
            if (this->G.types[curr_goal] == "Workstation") {
                next_goal = sample_endpiont();
            } else if (this->G.types[curr_goal] == "Endpoint") {
                next_goal = sample_workstation();
            } else {
                spdlog::error(
                    "Agent {}: Prior goal is not workstation or endpoint.",
                    agent_id);
                exit(-1);
            }
        }
    }
    // For non-warehouse, generate goals from free locations
    else {
        next_goal = this->sample_free_location();
    }

    return next_goal;
}
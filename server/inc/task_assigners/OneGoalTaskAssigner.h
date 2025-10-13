#pragma once
#include "task_assigners/BasicTaskAssigner.h"

// Assign each agent a goal location, and each agent only has one goal
// at a time. It is okay for the goals to have duplicates. With this task
// assigner, the planner is assumed to be able to plan target conflicts.
class OneGoalTaskAssigner : public BasicTaskAssigner {
public:
    OneGoalTaskAssigner() = default;
    OneGoalTaskAssigner(const SMARTGrid& G, int screen, int num_of_agents,
                        int seed, string task_file = "");
    void updateStartsAndGoals(vector<tuple<double, double, int>>& start_locs,
                              set<int> finished_tasks_id) override;

private:
    int genGoal(int curr_goal, int start_loc, int agent_id);
};
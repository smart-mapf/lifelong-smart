#pragma once
#include "heuristics/StateAStarNode.h"
#include "task_assigners/BasicTaskAssigner.h"

// Assign each agent a distinct goal location, and each agent only has one goal
// at a time. If the goal is occupied by another agent, the agent will go to a
// backup goal, which is an unoccupied free location. The agent will not change
// its goal until it reaches the goal.
class DistinctOneGoalTaskAssigner : public BasicTaskAssigner {
public:
    DistinctOneGoalTaskAssigner() = default;
    DistinctOneGoalTaskAssigner(
        const SMARTGrid &G,
        const shared_ptr<HeuristicTableBase> heuristic_table, int screen,
        int num_of_agents, int seed, string task_file = "");
    void updateStartsAndGoals(vector<tuple<double, double, int>> &start_locs,
                              set<int> finished_tasks_id) override;

private:
    // Remember the goals that the agents are attempting to go to, but are
    // occupied by other agents.
    vector<int> goal_buffer;
    int sampleUnoccupiedLoc(set<int> to_avoid, vector<int> candidates);
    int sampleBackupGoal(set<int> to_avoid, int curr_goal, int start_loc,
                         int agent_id);
    tuple<int, bool> genGoal(set<int> to_avoid, int curr_goal, int start_loc,
                             int agent_id);
};
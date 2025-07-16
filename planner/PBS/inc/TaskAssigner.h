#pragma once
#include "Graph.h"
#include "Task.h"
#include "common.h"

class TaskAssigner {
public:
    TaskAssigner(shared_ptr<Graph> graph, int screen, int simulation_window,
                 int num_of_agents)
        : graph(graph),
          screen(screen),
          simulation_window(simulation_window),
          num_of_agents(num_of_agents) {
        task_id = 0;
        goal_buffer.resize(num_of_agents, -1);
    }

    tuple<int, bool> genGoal(set<int> to_avoid, int curr_goal, int start_loc,
                             int agent_id);
    // int genOneGoalLoc(set<int> to_avoid, int curr_goal, int start_loc,
    //                   int agent_id);
    int sampleUnoccupiedLoc(set<int> to_avoid, vector<int> candidates);
    int sampleBackupGoal(set<int> to_avoid, int curr_goal, int start_loc,
                         int agent_id);
    void updateGoalLocations(vector<int> start_locations,
                             set<int> finished_tasks_id);
    vector<Task> getGoalLocations() const {
        return goal_locations;
    }

    set<int> getBackupTasks() const {
        return backup_tasks;
    }

private:
    vector<Task> goal_locations;  // goal locations for the agents
    shared_ptr<Graph> graph;      // graph representation of the map
    int simulation_window = 0;    // simulation window for the planner
    int screen = 0;               // screen output level

    // Remember the goals that the agents are attempting to go to, but are
    // occupied by other agents.
    vector<int> goal_buffer;
    set<int> backup_tasks;
    int num_of_agents;
    int task_id;
};
#pragma once
#include "task.h"
#include "common.h"
#include "graph.h"

class TaskAssigner {
public:
    TaskAssigner(shared_ptr<Graph> graph, int screen, int simulation_window,
                 int num_of_agents)
        : graph(graph),
          screen(screen),
          simulation_window(simulation_window),
          num_of_agents(num_of_agents) {
        task_id = 0;
    }

    tuple<int, bool> genGoal(set<int> to_avoid, int curr_goal, int start_loc,
                             int agent_id);
    void updateGoalLocations(vector<int> start_locations,
                             set<int> finished_tasks_id);
    vector<vector<Task>> getGoalLocations() const {
        return goal_locations;
    }


private:
    vector<vector<Task>> goal_locations;  // goal locations for the agents
    shared_ptr<Graph> graph;      // graph representation of the map
    int simulation_window = 0;    // simulation window for the planner
    int screen = 0;               // screen output level

    int num_of_agents;
    int task_id;
};
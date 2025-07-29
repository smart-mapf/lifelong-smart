#pragma once
#include "common.h"
#include "graph.h"
#include "task.h"

class TaskAssigner {
public:
    TaskAssigner(shared_ptr<Graph> graph, shared_ptr<RobotMotion> bot_motion,
                 int screen, int simulation_window, int num_of_agents)
        : graph(graph),
          bot_motion(bot_motion),
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
    shared_ptr<Graph> graph;              // graph representation of the map
    shared_ptr<RobotMotion> bot_motion;   // robot motion model
    int simulation_window = 0;            // simulation window for the planner
    int screen = 0;                       // screen output level

    int num_of_agents;
    int task_id;
};
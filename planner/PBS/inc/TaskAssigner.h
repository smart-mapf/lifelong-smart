#pragma once
#include "Graph.h"
#include "common.h"

class TaskAssigner {
public:
    TaskAssigner(shared_ptr<Graph> graph, int screen, int simulation_window)
        : graph(graph), screen(screen), simulation_window(simulation_window) {
    }

    int genGoal(set<int> to_avoid, int curr_goal, int start_loc);
    int genOneGoalLoc(set<int> to_avoid, int curr_goal, int start_loc);
    int sampleUnoccupiedLoc(set<int> to_avoid, vector<int> candidates);

private:
    shared_ptr<Graph> graph;    // graph representation of the map
    int simulation_window = 0;  // simulation window for the planner
    int screen = 0;             // screen output level

    // Remember the goals that the agents are attempting to go to, but are
    // occupied by other agents.
    vector<int> goal_buffer;
};
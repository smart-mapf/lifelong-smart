#pragma once
#include "Graph.h"
#include "TaskAssigner.h"
#include "common.h"

// Currently only works for undirected unweighted 4-nighbor grids
class Instance {
public:
    int screen = 0;
    shared_ptr<Graph> graph;                 // graph representation of the map
    shared_ptr<TaskAssigner> task_assigner;  // task assigner for the instance
    int simulation_window = 0;  // simulation window for the planner

    Instance() {
    }
    Instance(shared_ptr<Graph> graph, shared_ptr<TaskAssigner> task_assigner,
             int screen, int simulation_window);

    void setGoalLocations(const vector<Task>& goal_locations) {
        this->goal_locations = goal_locations;
    }

    vector<Task> getGoalTasks() const {
        return goal_locations;
    }

    vector<int> getGoalLocations() const {
        vector<int> goal_locs;
        for (const auto& task : goal_locations) {
            goal_locs.push_back(task.loc);
        }
        return goal_locs;
    }

    vector<int> getStartLocations() const {
        return start_locations;
    }

    int getTaskId() const {
        return task_id;
    }

    void printAgents() const;

    int getDefaultNumberOfAgents() const {
        return num_of_agents;
    }

    bool loadAgents(const json& mapf_instance);

private:
    int task_id = 0;
    int num_of_agents;
    vector<int> start_locations;
    vector<Task> goal_locations;

    // Class  SingleAgentSolver can access private members of Node
    friend class SingleAgentSolver;
};

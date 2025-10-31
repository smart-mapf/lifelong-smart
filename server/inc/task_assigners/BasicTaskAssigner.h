#pragma once
#include "heuristics/BasicHeuristicTable.h"
#include "utils/SMARTGraph.h"
#include "utils/Task.h"
#include "utils/common.h"

class BasicTaskAssigner {
public:
    BasicTaskAssigner() = default;
    BasicTaskAssigner(const SMARTGrid &G,
                      const shared_ptr<HeuristicTableBase> heuristic_table,
                      int screen, int num_of_agents, int seed,
                      string task_file = "");
    virtual void updateStartsAndGoals(
        vector<tuple<double, double, int>> &start_locs,
        set<int> finished_tasks_id) = 0;
    vector<State> getStarts() const {
        return starts;
    }
    vector<vector<Task>> getGoalLocations() const {
        return goal_locations;
    }
    set<int> getBackupTasks() const {
        return backup_tasks;
    }
    json getMAPFInstanceJSON() const;

protected:
    bool load_tasks(string task_file);

    void print_mapf_instance(vector<State> starts_,
                             vector<vector<Task>> goals_) const;
    int sample_workstation();
    int sample_endpiont();
    int sample_task_location();
    int sample_free_location();

    const SMARTGrid &G;

    const shared_ptr<HeuristicTableBase> heuristic_table;

    // Current start locations of the robots
    vector<State> starts;

    // Current goal locations of the robots
    vector<vector<Task>> goal_locations;

    // All available tasks for the robots. Only used if reading in tasks from
    // file.
    vector<list<Task>> tasks;

    // If true, tasks are randomly generated
    bool random_task = true;

    int screen;
    int num_of_agents;

    // A global task id counter to ensure each task has a unique id
    int task_id;

    // For random generation
    int seed;
    mt19937 gen;
    discrete_distribution<int> workstation_dist;
    discrete_distribution<int> endpoint_dist;
    discrete_distribution<int> task_location_dist;
    discrete_distribution<int> free_location_dist;

    // Remember the next goal type for each agent
    vector<CellType> next_goal_type;

    // Remember the tasks that are backup tasks, i.e., the agents are going to
    // these tasks because their original goals are occupied by other agents.
    set<int> backup_tasks;
};

// Assign each agent a sequence of goals, making sure that it has enough goals
// within the `simulation_window`, specified in timestpeps.
class WindowedTaskAssigner : public BasicTaskAssigner {
public:
    WindowedTaskAssigner() = default;
    WindowedTaskAssigner(const SMARTGrid &G,
                         const shared_ptr<HeuristicTableBase> heuristic_table,
                         int screen, int simulation_window, int num_of_agents,
                         int seed, string task_file = "");

    void updateStartsAndGoals(vector<tuple<double, double, int>> &start_locs,
                              set<int> finished_tasks_id) override;

private:
    int gen_next_goal(int agent_id, bool repeat_last_goal = false);

    // Simulation window in number of timesteps
    int simulation_window;
};
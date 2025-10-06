#include "backup_planners/SMARTGraph.h"
#include "backup_planners/Task.h"
#include "backup_planners/common.h"

class BasicTaskAssigner {
public:
    BasicTaskAssigner() = default;
    BasicTaskAssigner(const SMARTGrid& G, int screen, int num_of_agents,
                      int seed, string task_file = "");
    virtual void updateStartsAndGoals(
        vector<tuple<double, double, int>>& start_locs,
        set<int> finished_tasks_id) = 0;
    vector<State> getStarts() const {
        return starts;
    }
    vector<vector<Task>> getGoalLocations() const {
        return goal_locations;
    }
    json getMAPFInstanceJSON() const;

protected:
    bool load_tasks(string task_file);

    void print_mapf_instance(vector<State> starts_,
                             vector<vector<Task>> goals_) const;

    const SMARTGrid& G;

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

    // Remember the next goal type for each agent, "w" for workstation, "e" for
    // endpoint
    std::vector<string> next_goal_type;
};

class WindowedTaskAssigner : public BasicTaskAssigner {
public:
    WindowedTaskAssigner() = default;
    WindowedTaskAssigner(const SMARTGrid& G, int screen, int simulation_window,
                         int num_of_agents, int seed, string task_file = "");

    void updateStartsAndGoals(vector<tuple<double, double, int>>& start_locs,
                              set<int> finished_tasks_id) override;

private:
    int sample_workstation();
    int sample_endpiont();
    int gen_next_goal(int agent_id, bool repeat_last_goal = false);

    // Simulation window in number of timesteps
    int simulation_window;
};
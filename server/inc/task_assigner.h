#pragma once
#include <utility>
#include "user_map.h"

struct Task
{
    int id;
    int agent_id;
    std::pair<int, int> goal_position;
    std::pair<int, int> obj_position;
    int goal_orient=0;
    bool status=true; // false if finished, true otherwise
    Task(int id, int agent_id, std::pair<int, int> goal_position): id(id), agent_id(agent_id), goal_position(std::move(goal_position)) {
        obj_position=goal_position;
    }
    Task(int id, int agent_id, std::pair<int, int> goal_position, std::pair<int, int> obj_pos):
        id(id), agent_id(agent_id), goal_position(std::move(goal_position)), obj_position(std::move(obj_pos)) {}
};



class TaskManager
{
public:
    TaskManager(int num_agents, std::string& map_fname);
    virtual void getTask(std::vector<std::deque<std::shared_ptr<Task>>>& new_tasks);
    virtual void getTask(std::vector<std::deque<std::shared_ptr<Task>>>& new_tasks, std::vector<int> num_new_tasks);
    virtual ~TaskManager() = default;

protected:
    std::vector<std::deque<std::shared_ptr<Task>>> all_tasks;
    std::vector<int> finished_tasks;
    userMap user_map;
    int curr_task_idx  = 0;
    int num_robots_;
    int map_size_x_;
    int map_size_y_;
};
#pragma once
#include <utility>
#include "user_map.h"





class TaskManager
{
public:
    TaskManager(int num_agents, std::string& map_fname);
    // virtual void getTask(std::vector<std::deque<std::shared_ptr<Task>>>& new_tasks);
    // virtual void getTask(std::vector<std::deque<std::shared_ptr<Task>>>& new_tasks, std::vector<int> num_new_tasks);
    // virtual ~TaskManager() {};

protected:
    std::vector<int> finished_tasks;
    userMap user_map;
    int curr_task_idx  = 0;
    int num_robots_;
    int map_size_x_;
    int map_size_y_;
};
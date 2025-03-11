#pragma once
#include <utility>
#include <cassert>
#include "parser.h"

struct Task
{
    int id;
    int agent_id;
    std::pair<int, int> goal_position;
    int goal_orient=0;
    bool status=true; // false if finished, true otherwise
    Task(int id, int agent_id, std::pair<int, int> goal_position): id(id), agent_id(agent_id), goal_position(std::move(goal_position)) {}
};

class TaskManager
{
public:
    // TaskManager();
    TaskManager(int num_total_robots, int map_size_x, int map_size_y);
    void getRandomTasks(std::vector<std::deque<Task>>& new_tasks);
    void generateCharTask(int num_char = 3);
    void getCharTask(std::vector<std::deque<Task>>& new_tasks);
    bool isAgentFinished(int robot_id, std::pair<double, double> curr_pos) {
        //        std::cout << "finished_node_idx[robot_id]:" << robot_id << ": " << finished_node_idx[robot_id]
        //        << ";" << "plan size: " << graph[robot_id].size()-1 << std::endl;
        Task curr_goal = all_tasks[robot_id][finished_tasks[robot_id]+1];
        if (positionCompare(curr_pos, curr_goal.goal_position)) {
            finished_tasks[robot_id]+=1;
            return true;
        }
        return false;
    }


private:
    std::vector<std::deque<Task>> all_tasks;
    std::vector<int> finished_tasks;
    int curr_task_idx  = 0;
    int num_char_ = 0;
    int num_robots_;
    int map_size_x_;
    int map_size_y_;
};
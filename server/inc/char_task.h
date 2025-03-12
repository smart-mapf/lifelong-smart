#pragma once

#include "task_assigner.h"


class CharTask final : private TaskManager{
public:
    CharTask(int num_agents, std::string& map_fname);
    void getTask(std::vector<std::deque<std::shared_ptr<Task>>>& new_tasks);
    void generateTask(int num_char = 3);
    bool isAgentFinished(int robot_id, const std::pair<double, double>& curr_pos) {
        //        std::cout << "finished_node_idx[robot_id]:" << robot_id << ": " << finished_node_idx[robot_id]
        //        << ";" << "plan size: " << graph[robot_id].size()-1 << std::endl;
        std::shared_ptr<Task> curr_goal = all_tasks[robot_id][finished_tasks[robot_id]+1];
        if (positionCompare(curr_pos, curr_goal->goal_position)) {
            finished_tasks[robot_id]+=1;
            return true;
        }
        return false;
    }
    // ~CharTask() override = default;
private:
    int num_char_ = 0;
};
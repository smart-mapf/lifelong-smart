#include "task_assigner.h"

TaskManager::TaskManager(int num_agents, std::string& map_fname): user_map(map_fname) {
    num_robots_ = num_agents;
    // TODO@jingtian: check this
    map_size_x_ = user_map.num_of_cols;
    map_size_y_ = user_map.num_of_rows;

    finished_tasks.resize(num_robots_, -1);
}







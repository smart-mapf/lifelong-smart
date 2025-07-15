#include "Instance.h"

#include <algorithm>  // std::shuffle
#include <chrono>     // std::chrono::system_clock
#include <random>     // std::default_random_engine

// int RANDOM_WALK_STEPS = 100000;

Instance::Instance(shared_ptr<Graph> graph,
                   shared_ptr<TaskAssigner> task_assigner,
                   vector<Task> goal_locations, int screen, int task_id,
                   int simulation_window)
    : graph(graph),
      task_assigner(task_assigner),
      goal_locations(goal_locations),
      screen(screen),
      task_id(task_id),
      simulation_window(simulation_window) {
    // bool succ = loadMap();
    // // printMap();
    // if (!succ) {
    //     if (num_of_rows > 0 && num_of_cols > 0 && num_of_obstacles >= 0 &&
    //         num_of_obstacles <
    //             num_of_rows * num_of_cols)  // generate random grid
    //     {
    //         generateConnectedRandomGrid(num_of_rows, num_of_cols,
    //                                     num_of_obstacles);
    //         saveMap();
    //     } else {
    //         cerr << "Map file " << map_fname << " not found." << endl;
    //         exit(-1);
    //     }
    // }

    // succ = loadAgents();
    // if (!succ)
    // {
    // 	if (num_of_agents > 0)
    // 	{
    // 		generateRandomAgents(warehouse_width);
    // 		saveAgents();
    // 	}
    // 	else
    // 	{
    // 		cerr << "Agent file " << agent_fname << " not found." << endl;
    // 		exit(-1);
    // 	}
    // }
}

// void Instance::generateConnectedRandomGrid(int rows, int cols, int obstacles)
// {
//     cout << "Generate a " << rows << " x " << cols << " grid with " <<
//     obstacles
//          << " obstacles. " << endl;
//     int i, j;
//     num_of_rows = rows + 2;
//     num_of_cols = cols + 2;
//     map_size = num_of_rows * num_of_cols;
//     my_map.resize(map_size, false);
//     // Possible moves [WAIT, NORTH, EAST, SOUTH, WEST]
//     /*moves_offset[Instance::valid_moves_t::WAIT_MOVE] = 0;
//     moves_offset[Instance::valid_moves_t::NORTH] = -num_of_cols;
//     moves_offset[Instance::valid_moves_t::EAST] = 1;
//     moves_offset[Instance::valid_moves_t::SOUTH] = num_of_cols;
//     moves_offset[Instance::valid_moves_t::WEST] = -1;*/

//     // add padding
//     i = 0;
//     for (j = 0; j < num_of_cols; j++)
//         my_map[this->graph->linearizeCoordinate(i, j)] = true;
//     i = num_of_rows - 1;
//     for (j = 0; j < num_of_cols; j++)
//         my_map[this->graph->linearizeCoordinate(i, j)] = true;
//     j = 0;
//     for (i = 0; i < num_of_rows; i++)
//         my_map[this->graph->linearizeCoordinate(i, j)] = true;
//     j = num_of_cols - 1;
//     for (i = 0; i < num_of_rows; i++)
//         my_map[this->graph->linearizeCoordinate(i, j)] = true;

//     // add obstacles uniformly at random
//     i = 0;
//     while (i < obstacles) {
//         int loc = rand() % map_size;
//         if (addObstacle(loc)) {
//             printMap();
//             i++;
//         }
//     }
// }

// void Instance::saveInstance() {
//     std::ofstream myfile;
//     myfile.open("failed_instance.txt");
// }

// bool Instance::addObstacle(int obstacle) {
//     if (my_map[obstacle])
//         return false;
//     my_map[obstacle] = true;
//     int obstacle_x = this->graph->getRowCoordinate(obstacle);
//     int obstacle_y = this->graph->getColCoordinate(obstacle);
//     int x[4] = {obstacle_x, obstacle_x + 1, obstacle_x, obstacle_x - 1};
//     int y[4] = {obstacle_y - 1, obstacle_y, obstacle_y + 1, obstacle_y};
//     int start = 0;
//     int goal = 1;
//     while (start < 3 && goal < 4) {
//         if (x[start] < 0 || x[start] >= num_of_rows || y[start] < 0 ||
//             y[start] >= num_of_cols ||
//             my_map[this->graph->linearizeCoordinate(x[start], y[start])])
//             start++;
//         else if (goal <= start)
//             goal = start + 1;
//         else if (x[goal] < 0 || x[goal] >= num_of_rows || y[goal] < 0 ||
//                  y[goal] >= num_of_cols ||
//                  my_map[this->graph->linearizeCoordinate(x[goal], y[goal])])
//             goal++;
//         else if (isConnected(
//                      this->graph->linearizeCoordinate(x[start], y[start]),
//                      this->graph->linearizeCoordinate(
//                          x[goal],
//                          y[goal])))  // cannot find a path from start to goal
//         {
//             start = goal;
//             goal++;
//         } else {
//             my_map[obstacle] = false;
//             return false;
//         }
//     }
//     return true;
// }

// bool Instance::isConnected(int start, int goal) {
//     std::queue<int> open;
//     vector<bool> closed(map_size, false);
//     open.push(start);
//     closed[start] = true;
//     while (!open.empty()) {
//         int curr = open.front();
//         open.pop();
//         if (curr == goal)
//             return true;
//         for (int next : getNeighbors(curr)) {
//             if (closed[next])
//                 continue;
//             open.push(next);
//             closed[next] = true;
//         }
//     }
//     return false;
// }

// Update the goal locations
bool Instance::loadAgents(
    std::vector<std::tuple<double, double, int>>& start_locs,
    set<int> finished_tasks_id) {
    num_of_agents = static_cast<int>(start_locs.size());
    if (num_of_agents == 0) {
        cerr << "The number of agents should be larger than 0" << endl;
        exit(-1);
    }
    start_locations.resize(num_of_agents);
    if (goal_locations.size() != num_of_agents) {
        goal_locations.resize(num_of_agents, Task(-1, -1));
    }

    // Remove finished tasks from goal locations
    set<int> unfinished_goal_locs;  // For duplicate checking
    for (int i = 0; i < num_of_agents; i++) {
        if (finished_tasks_id.find(goal_locations[i].id) !=
            finished_tasks_id.end()) {
            goal_locations[i].id = -1;  // reset goal id
        } else {
            unfinished_goal_locs.insert(goal_locations[i].loc);
        }
    }

    // Generate new starts/goals
    for (int i = 0; i < num_of_agents; i++) {
        // Obtain the starts
        int row = static_cast<int>(std::get<0>(start_locs[i]));
        int col = static_cast<int>(std::get<1>(start_locs[i]));
        start_locations[i] = this->graph->linearizeCoordinate(row, col);

        // generate a goal for the agent if it does not have one
        if (goal_locations[i].id == -1) {
            spdlog::info("Agent {} has no goal, generating a new one.", i);
            int curr_goal = goal_locations[i].loc;
            goal_locations[i] =
                Task(this->task_id,
                     task_assigner->genGoal(unfinished_goal_locs, curr_goal,
                                            start_locations[i]));
            unfinished_goal_locs.insert(goal_locations[i].loc);
            this->task_id++;
        }
    }

    spdlog::info("Number of agents: {}", num_of_agents);
    spdlog::info("Number of goals: {}", goal_locations.size());
    spdlog::info("screen level: {}", this->screen);

    // Print the start and goal locations
    if (this->screen > 0) {
        spdlog::info("Start to goal locations:");
        for (int i = 0; i < num_of_agents; i++) {
            spdlog::info("Agent {} : S=({}, {}) ; G=({}, {})", i,
                         this->graph->getRowCoordinate(start_locations[i]),
                         this->graph->getColCoordinate(start_locations[i]),
                         this->graph->getRowCoordinate(goal_locations[i].loc),
                         this->graph->getColCoordinate(goal_locations[i].loc));
        }
        // cout << "Goal locations: ";
        // for (int i = 0; i < num_of_agents; i++) {
        //     cout << "a_" << i << ":("
        //          << this->graph->getRowCoordinate(goal_locations[i].loc) <<
        //          ","
        //          << this->graph->getColCoordinate(goal_locations[i].loc)
        //          << ", d_h="
        //          << this->graph
        //                 .d_heuristics[goal_locations[i].loc][start_locations[i]]
        //          << ") ";
        // }
        // cout << endl;
    }

    spdlog::info("Instance loaded");
    return true;
}

// int Instance::genGoal(set<int> to_avoid, int curr_goal, int agent_id) {
//     int goal = this->genOneGoalLoc(to_avoid, curr_goal, agent_id);
//     while (to_avoid.find(goal) != to_avoid.end() || goal == curr_goal ||
//            this->graph->getManhattanDistance(goal, start_locations[agent_id])
//            <
//                this->simulation_window) {
//         goal = this->genOneGoalLoc(to_avoid, curr_goal, agent_id);
//     }
//     return goal;
// }

void Instance::printAgents() const {
    for (int i = 0; i < num_of_agents; i++) {
        cout << "Agent" << i << " : S=("
             << this->graph->getRowCoordinate(start_locations[i]) << ","
             << this->graph->getColCoordinate(start_locations[i]) << ") ; G=("
             << this->graph->getRowCoordinate(goal_locations[i].loc) << ","
             << this->graph->getColCoordinate(goal_locations[i].loc) << ")"
             << endl;
    }
}

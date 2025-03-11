#include "task_assigner.h"

#include <boost/mpl/assert.hpp>

bool userMap::readMap(std::string& map_fname) {
    using namespace boost;
    using namespace std;
    ifstream myfile(map_fname.c_str());
    if (!myfile.is_open())
        return false;
    string line;
    tokenizer< char_separator<char> >::iterator beg;
    getline(myfile, line);
    if (line[0] == 't') // Nathan's benchmark
    {
        char_separator<char> sep(" ");
        getline(myfile, line);
        tokenizer< char_separator<char> > tok(line, sep);
        beg = tok.begin();
        beg++;
        num_of_rows = atoi((*beg).c_str()); // read number of rows
        getline(myfile, line);
        tokenizer< char_separator<char> > tok2(line, sep);
        beg = tok2.begin();
        beg++;
        num_of_cols = atoi((*beg).c_str()); // read number of cols
        getline(myfile, line); // skip "map"
    }
    map_size = num_of_cols * num_of_rows;
    my_map.resize(map_size, false);
    // read map (and start/goal locations)
    for (int i = 0; i < num_of_rows; i++) {
        getline(myfile, line);
        for (int j = 0; j < num_of_cols; j++) {
            my_map[i][j] = (line[j] != '.');
            if (line[j] == 'S') {
                // If station
                ;
            } else if (line[j] == 'E') {
                // If Pods
                ;
            }
        }
    }
    myfile.close();
    return true;
}

TaskManager::TaskManager(int num_total_robots, int map_size_x, int map_size_y) {
    num_robots_ = num_total_robots;
    map_size_x_ = map_size_x;
    map_size_y_ = map_size_y;
    all_tasks.resize(num_robots_);
    finished_tasks.resize(num_robots_, -1);
    generateCharTask();
    num_char_ = 3;
}

void TaskManager::generateCharTask(int num_char) {
    std::vector< std::pair<int, int> >char_C = {{1, 2}, {1, 3}, {1, 4}, {1, 5},
    {2, 1},
    {3, 1},
    {4, 1},
    {5, 2}, {5, 3}, {5, 4}, {5, 5}};
    std::vector< std::pair<int, int> >char_M = {{1, 1}, {1, 5},
    {2, 1}, {2, 2}, {2, 4}, {2, 5},
    {3, 1}, {3, 3}, {3, 5},
    {4, 1}, {4, 5}};
    std::vector< std::pair<int, int> >char_U = {{1, 1}, {1, 5},
    {2, 1}, {2, 5},
    {3, 1}, {3, 5},
    {4, 1}, {4, 5},
    {5, 2}, {5, 3}, {5, 4}};

    int offset_x = 0;
    int offset_y = 0;
    int task_id = 0;
    for (int i = 0; i < num_robots_; i++) {
        all_tasks[i].emplace_back(task_id, i, std::make_pair(char_C[i].second+offset_x, char_C[i].first+offset_y));
        task_id++;
    }
    for (int i = 0; i < num_robots_; i++) {
        all_tasks[i].emplace_back(task_id, i, std::make_pair(char_M[i].second+offset_x, char_M[i].first+offset_y));
        task_id++;
    }
    for (int i = 0; i < num_robots_; i++) {
        all_tasks[i].emplace_back(task_id, i, std::make_pair(char_U[i].second+offset_x, char_U[i].first+offset_y));
        task_id++;
    }

}

void TaskManager::getCharTask(std::vector<std::deque<Task>>& new_tasks) {
    new_tasks.resize(num_robots_);
    int task_increase_step = 1;
    for (int i = 0; i < num_robots_; i++) {
        // new_tasks[i].push_back();
        // assert(finished_tasks[i] <= curr_task_idx);
        // printf("For agent %d, its finished task is: %d\n", i, finished_tasks[i]);
        if (curr_task_idx > finished_tasks[i]) {
            task_increase_step = 0;
            break;
        }
    }
    curr_task_idx += task_increase_step;

    curr_task_idx = std::min(curr_task_idx, num_char_-1);
    // printf("curr_task_idx: %d\n", curr_task_idx);

    for (int i = 0; i < num_robots_; i++) {
        new_tasks[i].push_back(all_tasks[i][curr_task_idx]);
    }
}

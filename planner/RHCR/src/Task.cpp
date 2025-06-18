#include "Task.h"

std::ostream& operator<<(std::ostream& os, const Task& task) {
    os << "( loc : " << task.location << ", ori : " << task.orientation
       << ", wait_time : " << task.task_wait_time
       << ", hold_time : " << task.hold_time
       << ", is_parking : " << task.is_parking
       << ", movable : " << task.movable
       << ", id : " << task.id
       << " )";
    return os;
}

vector<list<Task>> read_task_vec(const std::string& fname, int num_of_drives) {
    auto tuple_vec = read_tuple_vec(fname, num_of_drives);
    vector<list<Task>> task_vec;
    for (int i = 0; i < num_of_drives; i++) {
        list<Task> task_list;
        for (auto& t : tuple_vec[i]) {
            // hold_time is assumed to not read in from file
            task_list.push_back(Task(std::get<0>(t), std::get<1>(t),
                                     std::get<2>(t), 0, -1, -1, false,
                                     static_cast<bool>(std::get<3>(t))));
        }
        task_vec.push_back(task_list);
    }
    return task_vec;
}

vector<tuple<int, int>> read_start_vec(const std::string& fname,
                                       int num_of_drives) {
    auto raw_int_vec = read_int_vec(fname, num_of_drives);
    vector<tuple<int, int>> start_vec;  // vector of (loc, ori)
    for (int i = 0; i < num_of_drives; i++) {
        auto start_tuple =
            make_tuple(raw_int_vec[i].front(), raw_int_vec[i].back());
        start_vec.push_back(start_tuple);
    }
    return start_vec;
}
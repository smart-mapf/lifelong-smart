#pragma once
#include "common.h"

struct Task {
    int id;
    int loc;
    orient ori;             // orientation of the task
    double task_wait_time;  // wait time of the task in seconds for MASS, in
                            // timestep for PIBT

    Task(int id, int loc, orient ori = orient::None,
         double task_wait_time = 0.0)
        : id(id), loc(loc), ori(ori), task_wait_time(task_wait_time) {
    }
};
#pragma once
#include "common.h"

struct Task {
    int id;
    int loc;
    orient ori;  // orientation of the task

    Task(int id, int loc, orient ori = orient::None)
        : id(id), loc(loc), ori(ori) {
    }
};
#pragma once
#include "common.h"

struct Task {
    int id;
    int loc;

    Task(int id, int loc) : id(id), loc(loc) {
    }
};
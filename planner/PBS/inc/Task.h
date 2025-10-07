#pragma once
#include "common.h"

struct Task {
    int id;
    int loc;

    Task() : id(-1), loc(-1) {
    }
    Task(int id, int loc) : id(id), loc(loc) {
    }
};
#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <regex>
#include <iomanip>  // For setting precision in double formatting
#include <cmath>

#define MAX_LOADS 1
#define MAX_TASKS 100
#define DEBUG

using namespace std;

const double EPS = 1e-8;


// Define a structure to hold coordinate points and time.
struct Point {
    int x, y;
    double time;
};

// Define a structure to represent each step an agent takes.
struct Step {
    int x, y, orientation;
    double time;
};

struct Pod {
    int x, y, orientation;
    int idx = -1;
    // true if assigned, false if free
    bool status = false;

    Pod(int x, int y, int orientation, int idx): x(x), y(y), orientation(orientation), idx(idx) {}
};

struct Station {
    int x, y, orientation;
    int idx = -1;
    bool status = false;
    Station(int x, int y, int orientation, int idx): x(x), y(y), orientation(orientation), idx(idx) {}
};

// Definition of the Action struct
struct Action {
    int robot_id;
    double time; // time that start an action
    double orientation;
    char type;  // 'M' for move, 'T' for turn, 'S' for station, 'P' for pod
    std::pair<double, double> start;
    std::pair<double, double> goal;
    int nodeID;
};

struct Task
{
    int id;
    int agent_id;
    std::pair<int, int> goal_position;
    std::pair<int, int> obj_position;
    int goal_orient=0;
    // Maybe find a better way?
    // For now, 0 is to station, 1 is to pod
    int flag = -1;
    int operate_obj_idx = -1;

    bool status=true; // false if finished, true otherwise
    Task(int id, int agent_id, std::pair<int, int> goal_position): id(id), agent_id(agent_id), goal_position(std::move(goal_position)) {
        obj_position=goal_position;
    }
    Task(int id, int agent_id, std::pair<int, int> goal_position, std::pair<int, int> obj_pos, int obj_idx, int flag):
        id(id), agent_id(agent_id), goal_position(std::move(goal_position)), obj_position(std::move(obj_pos)),
        operate_obj_idx(obj_idx), flag(flag) {}
};

inline bool positionCompare(std::pair<double, double> a, std::pair<int, int> b) {
    if (std::fabs(a.first - b.first) < EPS and std::fabs(a.second - b.second) < EPS) {
        return true;
    }
    return false;
}
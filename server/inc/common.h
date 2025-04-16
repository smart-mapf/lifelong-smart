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
#define MAX_TASKS 50
#define MAX_NUM_GOALS 1
#define NUM_GENRE 10
#define MAX_ENQUE 12
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
    int task_id = -1;
};

struct PickerTask
{
    int id;
    int genre;
    std::pair<int, int> goal_position;
    std::pair<int, int> obj_position;
    int operate_obj_idx = -1;

    bool status=true; // false if finished, true otherwise
    PickerTask(int id, int genre, std::pair<int, int> goal_position): id(id), genre(genre), goal_position(std::move(goal_position)) {
        obj_position=goal_position;
    }
};

enum MobileAction {
    DELIVER = 0, PICK = 1, NONE = 2
};

struct pair_hash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

struct MobileRobotTask
{
    int id;
    int agent_id;
    std::pair<int, int> goal_position;
    int goal_orient=0;

    MobileAction act = NONE;
    int operate_obj_idx = -1;
    int estimate_time = 0;
    bool status=true; // false if finished, true otherwise

    int station_id;
    // MobileRobotTask(int id, int agent_id, std::pair<int, int> goal_position): id(id), agent_id(agent_id), goal_position(std::move(goal_position)) {
    // }
    MobileRobotTask(int id, int agent_id, std::pair<int, int> goal_position, int obj_idx, MobileAction flag):
        id(id), agent_id(agent_id), goal_position(std::move(goal_position)), operate_obj_idx(obj_idx), act(flag) {
        // TODO:Find Station id given the goal location
        station_id = 0;
    }
};

inline bool positionCompare(std::pair<double, double> a, std::pair<int, int> b) {
    if (std::fabs(a.first - b.first) < EPS and std::fabs(a.second - b.second) < EPS) {
        return true;
    }
    return false;
}
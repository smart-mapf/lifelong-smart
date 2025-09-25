#pragma once
#include <cassert>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <queue>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "json.hpp"
#include "spdlog/spdlog.h"

#define MAX_LOADS 1
#define MAX_TASKS 100
#define MAX_NUM_GOALS 1
#define NUM_GENRE 10
#define MAX_ENQUE 5
// #define DEBUG

using namespace std;
using json = nlohmann::json;

const double EPS = 1e-8;
const int INF = numeric_limits<int>::max();

typedef pair<double, double> Location;

// Define a structure to hold coordinate points and time.
struct Point {
    int x, y;
    double time;
    int task_id = -1;  // Task ID associated with this point, if any

    Point() = default;
    Point(int x_, int y_, double w_, int task_id_ = -1)
        : x(x_), y(y_), time(w_), task_id(task_id_) {
    }
};

// Define a structure to represent each step an agent takes.
struct Step {
    int x, y, orientation;
    double time;
    int task_id = -1;  // Task ID associated with this step, if any
    Step() = default;
    Step(int x_, int y_, int orientation_, double t_, int task_id_ = -1)
        : x(x_), y(y_), orientation(orientation_), time(t_), task_id(task_id_) {
    }
};

struct Pod {
    int x, y, orientation;
    int idx = -1;
    // true if assigned, false if free
    bool status = false;

    Pod(int x, int y, int orientation, int idx)
        : x(x), y(y), orientation(orientation), idx(idx) {
    }
};

struct Station {
    int x, y, orientation;
    int idx = -1;
    bool status = false;
    Station(int x, int y, int orientation, int idx)
        : x(x), y(y), orientation(orientation), idx(idx) {
    }
};

struct PickerTask {
    int id;
    int genre;
    std::pair<int, int> goal_position;
    std::pair<int, int> obj_position;
    int operate_obj_idx = -1;

    bool status = true;  // false if finished, true otherwise
    PickerTask(int id, int genre, std::pair<int, int> goal_position)
        : id(id), genre(genre), goal_position(std::move(goal_position)) {
        obj_position = goal_position;
    }
};

enum MobileAction {
    // DELIVER = 0, PICK = 1, NONE = 2
    NONE = 0,
    PICK = 1,
    DELIVER = 2,
    DONE = 3
};

struct pair_hash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

struct MobileRobotTask {
    int id;
    int agent_id;
    std::pair<int, int> goal_position;
    int goal_orient = 0;

    MobileAction status = NONE;
    int operate_obj_idx = -1;
    int estimate_time = 0;
    // bool status=true; // false if finished, true otherwise

    int picker_robot_id = -1;
    std::pair<int, int> station_position;
    // MobileRobotTask(int id, int agent_id, std::pair<int, int> goal_position):
    // id(id), agent_id(agent_id), goal_position(std::move(goal_position)) {
    // }
    MobileRobotTask(int id, int agent_id, std::pair<int, int> goal_position,
                    int obj_idx, MobileAction flag, int picker_id,
                    std::pair<int, int> station_position)
        : id(id),
          agent_id(agent_id),
          goal_position(std::move(goal_position)),
          operate_obj_idx(obj_idx),
          status(flag),
          picker_robot_id(picker_id),
          station_position(station_position) {
    }

    std::pair<int, int> get_goal_position() const {
        if (status == PICK) {
            return goal_position;
        } else if (status == DELIVER) {
            return station_position;
        } else {
            std::cerr << "Invalid action" << std::endl;
            exit(1);
        }
    }
};

// Definition of the Action struct
struct Action {
    int robot_id;
    double time;  // time that start an action
    double orientation;
    char type;  // 'M' for move, 'T' for turn, 'S' for station, 'P' for pod
    std::pair<double, double> start;
    std::pair<double, double> goal;
    int nodeID;
    std::shared_ptr<MobileRobotTask> task_ptr = nullptr;
    int task_id = -1;  // Task ID associated with the completion of this action

    // default constructor
    Action()
        : robot_id(-1),
          time(0.0),
          orientation(0.0),
          type('M'),  // Default to 'M' for move
          start({0.0, 0.0}),
          goal({0.0, 0.0}),
          nodeID(-1),
          task_ptr(nullptr),
          task_id(-1) {
    }

    // Assignment operator
    Action& operator=(const Action& other) {
        robot_id = other.robot_id;
        time = other.time;
        orientation = other.orientation;
        type = other.type;
        start = other.start;
        goal = other.goal;
        nodeID = other.nodeID;
        task_ptr = other.task_ptr;  // Shared pointer assignment
        task_id = other.task_id;
        return *this;
    }

    // Copy constructor
    Action(const Action& other)
        : robot_id(other.robot_id),
          time(other.time),
          orientation(other.orientation),
          type(other.type),
          start(other.start),
          goal(other.goal),
          nodeID(other.nodeID),
          task_ptr(other.task_ptr),  // Shared pointer copy
          task_id(other.task_id) {
    }
};

inline bool positionCompare(std::pair<double, double> a,
                            std::pair<int, int> b) {
    if (std::fabs(a.first - b.first) < EPS and
        std::fabs(a.second - b.second) < EPS) {
        return true;
    }
    return false;
}

template <typename T>
T twoPointDistance(std::pair<T, T> a, std::pair<T, T> b) {
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}

// bool congested(
//     const std::vector<std::vector<std::tuple<int, int, double, int>>>&
//         new_plan);
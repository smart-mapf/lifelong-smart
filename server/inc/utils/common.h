#pragma once
#include <boost/algorithm/string.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <mutex>
#include <numeric>
#include <queue>
#include <random>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "spdlog/spdlog.h"
#include "utils/enums.h"
#include "utils/json.hpp"

using boost::unordered_map;
using boost::unordered_set;
using boost::heap::compare;
using boost::heap::fibonacci_heap;
using boost::program_options::variables_map;
using json = nlohmann::json;

using std::cerr;
using std::cin;
using std::cout;
using std::deque;
using std::discrete_distribution;
using std::endl;
using std::fixed;
using std::hash;
using std::list;
using std::lock_guard;
using std::make_pair;
using std::make_shared;
using std::make_tuple;
using std::map;
using std::max;
using std::min;
using std::mt19937;
using std::mutex;
using std::numeric_limits;
using std::ofstream;
using std::ostream;
using std::pair;
using std::queue;
using std::set;
using std::setprecision;
using std::shared_ptr;
using std::size_t;
using std::string;
using std::tie;
using std::to_string;
using std::tuple;
using std::vector;

#define MAX_ENQUE 5
#define INTERVAL_MAX 10000
#define EPS 1e-8
#define WEIGHT_MAX INT_MAX / 2

typedef tuple<int, int, int, int, bool> Constraint;
typedef tuple<int, int, int, int, int> Conflict;
// [t_min, t_max), have conflicts or not
typedef tuple<int, int, bool> Interval;
typedef pair<double, double> Location;
typedef vector<tuple<string, int, double, string, pair<double, double>,
                     pair<double, double>, int>>
    SIM_PLAN;

ostream& operator<<(ostream& os, const Constraint& constraint);

ostream& operator<<(ostream& os, const Conflict& conflict);

ostream& operator<<(ostream& os, const Interval& interval);

// According to https://stackoverflow.com/a/40854664, we need to add a
// operator<<() for vector<int> so that boost can automatically cast the
// default vector parameter value. We need this specifically for the
// `station_wait_times` command line argument.
namespace std {
ostream& operator<<(ostream& os, const vector<int>& vec);
}

vector<list<int>> read_int_vec(string fname, int team_size);
vector<list<tuple<int, int, int, int>>> read_tuple_vec(string fname,
                                                       int team_size);

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

struct pair_hash {
    size_t operator()(const pair<int, int>& p) const {
        return hash<int>()(p.first) ^ (hash<int>()(p.second) << 1);
    }
};

struct MobileRobotTask {
    int id;
    int agent_id;
    pair<int, int> goal_position;
    int goal_orient = 0;

    MobileAction status = NONE;
    int operate_obj_idx = -1;
    int estimate_time = 0;
    // bool status=true; // false if finished, true otherwise

    int picker_robot_id = -1;
    pair<int, int> station_position;
    MobileRobotTask(int id, int agent_id, pair<int, int> goal_position,
                    int obj_idx, MobileAction flag, int picker_id,
                    pair<int, int> station_position)
        : id(id),
          agent_id(agent_id),
          goal_position(std::move(goal_position)),
          operate_obj_idx(obj_idx),
          status(flag),
          picker_robot_id(picker_id),
          station_position(station_position) {
    }

    pair<int, int> get_goal_position() const {
        if (status == PICK) {
            return goal_position;
        } else if (status == DELIVER) {
            return station_position;
        } else {
            cerr << "Invalid action" << endl;
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
    Location start;
    Location goal;
    int nodeID;
    shared_ptr<MobileRobotTask> task_ptr = nullptr;
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

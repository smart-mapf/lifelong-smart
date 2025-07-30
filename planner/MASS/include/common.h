#pragma once

#include <spdlog/spdlog.h>

#include <boost/filesystem.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/pairing_heap.hpp>
#include <boost/tokenizer.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <cfloat>
#include <chrono>
#include <climits>
#include <ctime>
#include <fstream>
#include <iomanip>   // std::setprecision
#include <iostream>  // std::cout, std::fixed
#include <json.hpp>
#include <list>
#include <random>
#include <set>
#include <tuple>
#include <vector>

using boost::char_separator;
using boost::tokenizer;
using boost::unordered_map;
using boost::unordered_set;
using boost::heap::compare;
using boost::heap::fibonacci_heap;
using boost::heap::pairing_heap;
using std::cerr;
using std::clock;
using std::cout;
using std::endl;
using std::get;
using std::getline;
using std::ifstream;
using std::list;
using std::make_pair;
using std::make_shared;
using std::make_tuple;
using std::map;
using std::max;
using std::min;
using std::mt19937;
using std::ofstream;
using std::pair;
using std::set;
using std::shared_ptr;
using std::stack;
using std::string;
using std::tie;
using std::to_string;
using std::tuple;
using std::vector;

using json = nlohmann::json;

// Hyper-parameters
#define INF DBL_MAX / 2
#define EPS 0.0001
#define CELL_DIS 1.0
#define TIME_STEP_SIZE 0.05

#define WINDOW_SIZE INF
#define WEIGHT_MAX INT_MAX / 2

#define NUM_ORIENT 4
#define CONTROL_POINTS_NUM 30
// #define V_MIN 0.0
// #define V_MAX 2.0
// #define A_MAX 2.0
// #define ROTATE_COST 2.0
// #define TURN_BACK_COST 3.6

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> time_s;

// // Calculates the time to accelerate from zero to max_speed or vice versa.
// inline double timeToMaxSpeed() {
//     return V_MAX / A_MAX;
// }

// // Calculates the distance covered during acceleration or deceleration from
// zero
// // to max_speed or vice versa.
// inline int distanceDuringAcceleration() {
//     double time = timeToMaxSpeed();
//     // Using equation: s = ut + 0.5 * a * t^2 (where u is initial speed,
//     which
//     // is 0 here)
//     return (int)(0.5 * A_MAX * time * time);
// }

enum orient { North = 0, East = 1, South = 2, West = 3, None = 4 };

enum Action { turnLeft, turnRight, turnBack, forward, none, wait };

// const double travel_cost[] = {0.60, 1.45, 0.8, 0.60, 0.5};

// const double arr_cell_cost[] = {2.8284, 4.0000, 4.8990, 5.6569,
//                                 6.3246, 6.9282, 7.4833, 8.0000};

// inline double arrLowerBound(size_t step) {
//     double total_t;
//     size_t length = step + 2;
//     if (step > 2 * distanceDuringAcceleration()) {
//         total_t = 2 * timeToMaxSpeed() +
//                   ((length - 2 * distanceDuringAcceleration() - 1) / V_MAX);
//     } else {
//         total_t = 2 * sqrt((length - 1) / A_MAX);
//     }
//     return total_t;
// }
/**
 * @brief This structure is used to manage the time intervals
 * during the partial node expansion.
 **/
struct IntervalEntry {
    double t_min = -1;
    double t_max = -1;
    double f = 0;
    int location;
    int interval_idx;
    size_t step;
    std::shared_ptr<IntervalEntry> prev_entry = nullptr;
    IntervalEntry() = default;
    IntervalEntry(double min, double max, int loc, int idx, size_t stp)
        : t_min(min), t_max(max), location(loc), interval_idx(idx), step(stp) {
    }
};

class IntervalCompare {
public:
    bool operator()(const std::shared_ptr<IntervalEntry>& N1,
                    const std::shared_ptr<IntervalEntry>& N2) {
        if (N1->f > N2->f) {
            return true;
        } else {
            return false;
        }
    }
};

typedef std::priority_queue<std::shared_ptr<IntervalEntry>,
                            std::vector<std::shared_ptr<IntervalEntry>>,
                            IntervalCompare>
    IntervalQueue;

typedef std::deque<std::shared_ptr<IntervalEntry>> IntervalSeq;

struct TimeInterval {
    double t_min;
    double t_max;
    int id;
    int agent_id;
    TimeInterval() {
        t_min = -1;
        t_max = -1;
        id = -1;
        agent_id = -1;
    }
    TimeInterval(double s, double e, int interval_id)
        : t_min(s), t_max(e), id(interval_id), agent_id(-1) {
    }
};

struct PathEntry {
    int location = -1;
    orient o = orient::East;
    double arrival_time = 0;       // arrival time of the head of the vehicle
    double leaving_time_tail = 0;  // leaving time of the tail of the vehicle;
    PathEntry() = default;
    PathEntry(int loc, double t_min, double t_max)
        : location(loc), arrival_time(t_min), leaving_time_tail(t_max) {
    }
    PathEntry(int loc, orient o, double t_min, double t_max)
        : location(loc), o(o), arrival_time(t_min), leaving_time_tail(t_max) {
    }
};

typedef std::deque<PathEntry> Path;

std::ostream& operator<<(std::ostream& os, const Path& path);

bool isSamePath(const Path& p1, const Path& p2);

typedef vector<vector<TimeInterval>>
    ReservationTable;  // vector: vertex -> ordered list of occupied time
                       // intervals

typedef std::deque<std::pair<int, double>> TimedPath;

struct MotionNode {
    std::vector<double> control_points;
    Path local_path;
    double optimal_T = -1;
    double start_t = -1;
    double end_t = -1;
    Action type = Action::none;
};

typedef vector<std::shared_ptr<MotionNode>> MotionInfo;

struct Node {
    Node() {
    }
    Node(int goal_id, int current_point, orient curr_o, int interval_index,
         double arr_min, double arr_max, double g_val, double h_val,
         double win_size, Action prev_act, const std::shared_ptr<Node>& parent,
         std::shared_ptr<MotionNode> bezier_solution)
        : goal_id(goal_id),
          current_point(current_point),
          curr_o(curr_o),
          interval_index(interval_index),
          arrival_time_min(arr_min),
          arrival_time_max(arr_max),
          g(g_val),
          h(h_val),
          prev_action(prev_act),
          parent(parent),
          bezier_solution(bezier_solution) {
        // One-shot objective
        if (win_size <= 0)
            f = h + g;
        // Windowed objective
        else
            f = max(g, win_size) + h;
    }
    bool is_expanded = false;
    int current_point;
    Action prev_action;
    orient curr_o;
    int goal_id = 0;

    int interval_index;
    double arrival_time_min;  // arrival time of the head of the vehicle
    double arrival_time_max;  // arrival time of the head of the vehicle

    double g = 0;  //
    double f = 0;  // f = h + g;
    double h = 0;  //

    std::shared_ptr<MotionNode> bezier_solution = nullptr;
    std::shared_ptr<Node> parent = nullptr;
    IntervalQueue partial_intervals;

    bool estimated = false;  // whether the f val is estimated
};

class NodeCompare {
public:
    bool operator()(const std::shared_ptr<Node>& N1,
                    const std::shared_ptr<Node>& N2) {
        if (N1->f > N2->f) {
            return true;
        } else {
            return false;
        }
    }
};

typedef vector<Node> Successors;

struct NodeEqual {
    bool operator()(const std::shared_ptr<Node>& lhs,
                    const std::shared_ptr<Node>& rhs) const {
        return lhs->current_point == rhs->current_point and
               lhs->curr_o == rhs->curr_o and
               lhs->interval_index == rhs->interval_index and
               lhs->goal_id == rhs->goal_id;
    }
};

struct NodeHash {
    size_t operator()(const std::shared_ptr<Node>& key) const {
        size_t hashA = std::hash<int>()(key->current_point);
        size_t hashB = std::hash<int>()(key->curr_o);
        size_t hashC = std::hash<int>()(key->interval_index);
        size_t hashD = std::hash<int>()(key->goal_id);
        size_t hashE = std::hash<int>()(key->estimated);

        // size_t combined = (hashA << 8) + (hashC << 2) + hashB + hashD +
        //                   hashE;
        size_t combined =
            hashA ^ (hashB << 1) ^ (hashC << 2) ^ (hashD << 3) ^ (hashE << 4);
        return combined;
    }
};

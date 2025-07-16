#pragma once

#include <tuple>
#include <list>
#include <vector>
#include <set>
#include <ctime>
#include <fstream>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include <chrono>
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/tokenizer.hpp>
#include <boost/filesystem.hpp>
#include <spdlog/spdlog.h>
#include <random>
#include <json.hpp>

using boost::heap::pairing_heap;
using boost::heap::compare;
using boost::unordered_map;
using boost::unordered_set;
using std::vector;
using std::list;
using std::set;
using std::get;
using std::tuple;
using std::make_tuple;
using std::pair;
using std::make_pair;
using std::tie;
using std::min;
using std::max;
using std::shared_ptr;
using std::make_shared;
using std::clock;
using std::cout;
using std::endl;
using std::ofstream;
using std::cerr;
using std::string;
using boost::char_separator;
using boost::tokenizer;
using boost::heap::fibonacci_heap;
using std::cerr;
using std::getline;
using std::ifstream;
using std::map;
using std::mt19937;
using std::set;
using std::shared_ptr;
using std::stack;
using std::to_string;

using json = nlohmann::json;

// Hyper-parameters
#define INF 10000.0
#define EPS 0.0001
#define CELL_DIS 1.0
#define TIME_STEP_SIZE 0.05

#define WINDOW_SIZE INF
#define WEIGHT_MAX INT_MAX / 2


#define NUM_ORIENT 4
#define CONTROL_POINTS_NUM 30
#define V_MIN 0.0
#define V_MAX 2.0
#define A_MAX 0.5
#define ROTATE_COST 1.0
#define TURN_BACK_COST 1.8

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> time_s;

// Calculates the time to accelerate from zero to max_speed or vice versa.
inline double timeToMaxSpeed() {
    return V_MAX / A_MAX;
}

// Calculates the distance covered during acceleration or deceleration from zero to max_speed or vice versa.
inline int distanceDuringAcceleration() {
    double time = timeToMaxSpeed();
    // Using equation: s = ut + 0.5 * a * t^2 (where u is initial speed, which is 0 here)
    return (int) (0.5 * A_MAX * time * time);
}

enum orient {
    North = 0,
    East = 1,
    South = 2,
    West = 3,
    None = 4
};

enum Action{
    turnLeft,
    turnRight,
    turnBack,
    forward,
    none,
    wait
};

const double travel_cost[] = {0.60, 1.45, 0.8, 0.60, 0.5};

const double arr_cell_cost[] = {2.8284, 4.0000, 4.8990, 5.6569, 6.3246, 6.9282, 7.4833, 8.0000};

inline double arrLowerBound(size_t step)
{
    double total_t;
    size_t length = step + 2;
    if (step > 2*distanceDuringAcceleration()) {
        total_t = 2*timeToMaxSpeed() + ((length - 2*distanceDuringAcceleration()-1)/V_MAX);
    } else {
        total_t = 2*sqrt((length-1)/A_MAX);
    }
    return total_t;
}
/**
 * @brief This structure is used to manage the time intervals
 * during the partial node expansion.
 **/
struct IntervalEntry
{
    double t_min = -1;
    double t_max = -1;
    double f = 0;
    int location;
    int interval_idx;
    size_t step;
    std::shared_ptr<IntervalEntry> prev_entry = nullptr;
    IntervalEntry() = default;
    IntervalEntry(double min, double max, int loc, int idx, size_t stp)
            : t_min(min), t_max(max), location(loc), interval_idx(idx), step(stp) {}
};

class IntervalCompare
{
public:
    bool operator() (const std::shared_ptr<IntervalEntry>& N1, const std::shared_ptr<IntervalEntry>& N2) {
        if (N1->f > N2->f) {
            return true;
        } else {
            return false;
        }
    }
};

typedef std::priority_queue<std::shared_ptr<IntervalEntry>,
        std::vector<std::shared_ptr<IntervalEntry>>, IntervalCompare> IntervalQueue;

typedef std::deque< std::shared_ptr<IntervalEntry> > IntervalSeq;

struct TimeInterval
{
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
    TimeInterval(double s, double e, int interval_id) :
        t_min(s), t_max(e), id(interval_id), agent_id(-1) {}
};

struct PathEntry
{
	int location = -1;
    orient o = orient::East;
    double arrival_time = 0; // arrival time of the head of the vehicle
    double leaving_time_tail = 0; // leaving time of the tail of the vehicle;
    PathEntry() = default;
    PathEntry(int loc, double t_min, double t_max): location(loc), arrival_time(t_min), leaving_time_tail(t_max) {}
    PathEntry(int loc, orient o, double t_min, double t_max): location(loc), o(o),arrival_time(t_min), leaving_time_tail(t_max) {}
};

typedef std::deque<PathEntry> Path;

std::ostream& operator<<(std::ostream& os, const Path& path);

bool isSamePath(const Path& p1, const Path& p2);

typedef vector<vector<TimeInterval>> ReservationTable; // vector: vertex -> ordered list of occupied time intervals

typedef std::deque<std::pair<int, double>> TimedPath;

struct MotionNode
{
    std::vector<double> control_points;
    Path local_path;
	double optimal_T = -1;
    double start_t = -1;
    double end_t = -1;
    Action type = Action::none;
};

typedef vector<std::shared_ptr<MotionNode>> MotionInfo;
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

inline bool positionCompare(std::pair<double, double> a, std::pair<int, int> b) {
    if (std::fabs(a.first - b.first) < EPS and std::fabs(a.second - b.second) < EPS) {
        return true;
    }
    return false;
}
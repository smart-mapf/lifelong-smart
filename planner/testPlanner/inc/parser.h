#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <regex>
#include <iomanip>  // For setting precision in double formatting

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

// Definition of the Action struct
struct Action {
    int robot_id;
    double time; // time that start an action
    double orientation;
    char type;  // 'M' for move, 'T' for turn
    std::pair<double, double> start;
    std::pair<double, double> goal;
    int nodeID;
};

enum PlanType {
    DISCRETE,
    CONTINUOUS
};

int getOrientation(int x1, int y1, int x2, int y2);
void processAgentActions(const vector<Point>& points, vector<Step>& steps, int agentId);
vector<Point> parseLine(const string& line);
std::vector<std::vector<Action>> processActions(const std::vector<std::vector<Step>>& raw_steps, bool flipped_coord);
bool parseEntirePlan(const std::string& input_file, std::vector<std::vector<Action>>& plans,
                     double& raw_cost, bool flipped_coord = true, PlanType file_type = PlanType::DISCRETE);
void showActionsPlan(std::vector<std::vector<Action>>& plans);
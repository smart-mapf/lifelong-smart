#pragma once
#include "common.h"

enum PlanType {
    DISCRETE,
    CONTINUOUS
};

int getOrientation(int x1, int y1, int x2, int y2);
void processAgentActions(const vector<Point>& points, vector<Step>& steps, int currentOrientation, int agentId);
vector<Point> parseLine(const string& line);
std::vector<std::vector<Action>> processActions(const std::vector<std::vector<Step>>& raw_steps, bool flipped_coord);
bool parseEntirePlan(const std::string& input_file, std::vector<std::vector<Action>>& plans,
                     double& raw_cost, bool flipped_coord = true, PlanType file_type = PlanType::DISCRETE);
void showActionsPlan(std::vector<std::vector<Action>>& plans);
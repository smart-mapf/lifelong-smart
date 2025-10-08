#pragma once
#include "utils/common.h"

class PlanParser {
public:
    PlanParser(int screen = 0) : screen(screen) {
    }
    int getOrientation(int x1, int y1, int x2, int y2);
    void AgentPathToSteps(const vector<Point>& points, vector<Step>& steps,
                          int currentOrientation, int agentId);
    vector<vector<Action>> StepsToActions(const vector<vector<Step>>& raw_steps,
                                          bool flipped_coord);

    void showPoints(int agentId, const vector<Point>& points);
    void showSteps(const vector<vector<Step>>& raw_steps);
    void showActions(const vector<vector<Action>>& plans);
    int screen;
};

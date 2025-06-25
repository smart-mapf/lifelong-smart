#pragma once
#include "common.h"

class PlanParser {
public:
    PlanParser(int screen = 0) : screen(screen) {}
    int getOrientation(int x1, int y1, int x2, int y2);
    void AgentPathToSteps(const vector<Point>& points, vector<Step>& steps,
                          int currentOrientation, int agentId);
    std::vector<std::vector<Action>> StepsToActions(
        const std::vector<std::vector<Step>>& raw_steps, bool flipped_coord);

    void showPoints(int agentId, const vector<Point>& points);
    void showSteps(const std::vector<std::vector<Step>>& raw_steps);
    void showActions(const std::vector<std::vector<Action>>& plans);
    int screen;
};

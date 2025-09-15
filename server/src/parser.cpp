#include "parser.h"

// Function to determine the orientation based on current and next position.
int PlanParser::getOrientation(int x1, int y1, int x2, int y2) {
    if (x2 == x1) {
        return y2 > y1 ? 1 : 3;  // North or South
    } else {
        return x2 > x1 ? 2 : 0;  // East or West
    }
}

void PlanParser::showPoints(int agentId,
                            const vector<Point>& points) {  // Print points
    cout << "Points of Agent " << agentId << endl;
    for (const auto& point : points) {
        cout << "(" << point.x << ", " << point.y << "), "
             << "time: " << point.time << ", task_id: " << point.task_id
             << endl;
    }
}

void PlanParser::showSteps(const vector<vector<Step>>& raw_steps) {
    // Print steps
    cout << "Raw Steps" << endl;
    for (size_t i = 0; i < raw_steps.size(); i++) {
        printf("Step of agent: %lu\n", i);
        for (const auto& tmp_step : raw_steps[i]) {
            cout << "(" << tmp_step.x << ", " << tmp_step.y << "), "
                 << tmp_step.orientation << ", time: " << tmp_step.time
                 << ", task_id: " << tmp_step.task_id << endl;
        }
        printf("\n");
    }
}

void PlanParser::showActions(const vector<vector<Action>>& plans) {
    // Print actions
    cout << "Processed Actions" << endl;
    for (size_t i = 0; i < plans.size(); i++) {
        printf("Action of agent: %lu\n", i);
        for (int j = 0; j < plans[i].size(); j++) {
            auto& action = plans[i][j];
            cout << "        {" << action.robot_id << ", " << action.time
                 << ", " << fixed << setprecision(1) << action.orientation
                 << ", '" << action.type << "', {" << action.start.first << ", "
                 << action.start.second << "}, {" << action.goal.first << ", "
                 << action.goal.second << "}, "
                 << "t = " << action.time << ", "
                 << "nodeid = " << action.nodeID << ", "
                 << "taskid = " << action.task_id << "}," << endl;
        }
        printf("\n");
    }
}

// Processes the actions of an agent, calculating necessary steps, including
// orientation changes.
void PlanParser::AgentPathToSteps(const vector<Point>& points,
                                  vector<Step>& steps, int currentOrientation,
                                  int agentId) {
    steps.clear();

    if (this->screen > 1)
        showPoints(agentId, points);

    // int currentOrientation = 0; // Assume initial orientation is West.
    // double currentTime = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        double currentTime = points[i].time;  // Use the time from the point.
        if (i == 0) {
            // Record the initial position and orientation.
            steps.push_back(Step(points[i].x, points[i].y, currentOrientation,
                                 currentTime, points[i].task_id));
        } else {
            double prevTime = points[i - 1].time;
            // Determine the necessary orientation for the next move.
            int neededOrientation = getOrientation(
                points[i - 1].x, points[i - 1].y, points[i].x, points[i].y);
            if (neededOrientation != currentOrientation) {
                // Change orientation without advancing time by adding an
                // additional step on the previous timestep with the needed
                // orientation.
                // Need to rotate 180 degrees
                if (abs(neededOrientation - currentOrientation) == 2) {
                    // If the needed orientation is opposite to the current one,
                    // we need to add a step to turn around.
                    int mid_ori = (currentOrientation + 1) % 4;
                    steps.push_back(Step(points[i - 1].x, points[i - 1].y,
                                         mid_ori, prevTime));
                    steps.push_back(Step(points[i - 1].x, points[i - 1].y,
                                         neededOrientation, prevTime));
                } else {
                    steps.push_back(Step(points[i - 1].x, points[i - 1].y,
                                         neededOrientation, prevTime));
                }

                currentOrientation = neededOrientation;
            }
            // Move to the next position
            steps.push_back(Step(points[i].x, points[i].y, currentOrientation,
                                 currentTime, points[i].task_id));
        }
    }
}

vector<vector<Action>> PlanParser::StepsToActions(
    const vector<vector<Step>>& raw_steps, bool flipped_coord) {
    if (this->screen > 1)
        showSteps(raw_steps);

    vector<vector<Action>> plans;
    int node_id = 0;
    for (size_t i = 0; i < raw_steps.size(); ++i) {
        vector<Action> processedActions;

        if (raw_steps[i].empty()) {
            // If there are no steps for this agent, skip to the next one.
        }

        // Special cases:
        // 1. If the agent has only one step, that is the goal. We need to add a
        //    special goal action.
        // 2. If the agent has more than one step, but the first step is a
        //    goal. Then we need to process the first step then the rest.
        else if (raw_steps[i][0].task_id >= 0) {
            Action goalAction;
            goalAction.robot_id = (int)i;
            goalAction.time = raw_steps[i][0].time;
            if (flipped_coord) {
                goalAction.start.first = raw_steps[i][0].y;
                goalAction.start.second = raw_steps[i][0].x;
            } else {
                goalAction.start.first = raw_steps[i][0].x;
                goalAction.start.second = raw_steps[i][0].y;
            }
            goalAction.goal = goalAction.start;  // Goal is the same as start
            goalAction.orientation = raw_steps[i][0].orientation;
            goalAction.type = 'S';  // Station
            goalAction.nodeID = node_id;
            node_id++;
            goalAction.task_id = raw_steps[i][0].task_id;
            processedActions.push_back(goalAction);
            // plans.push_back(processedActions);
            // continue;
        }

        // More than 2 steps are available, we need to process them
        if (raw_steps[i].size() > 1) {
            for (size_t j = 1; j < raw_steps[i].size(); j++) {
                double prev_step_x, prev_step_y, curr_step_x, curr_step_y;
                if (flipped_coord) {
                    prev_step_x = raw_steps[i][j - 1].y;
                    prev_step_y = raw_steps[i][j - 1].x;
                    curr_step_x = raw_steps[i][j].y;
                    curr_step_y = raw_steps[i][j].x;
                } else {
                    prev_step_x = raw_steps[i][j - 1].x;
                    prev_step_y = raw_steps[i][j - 1].y;
                    curr_step_x = raw_steps[i][j].x;
                    curr_step_y = raw_steps[i][j].y;
                }
                // cout << "Processing step " << j << " for agent " << i
                //      << ", prev: (" << prev_step_x << ", " << prev_step_y
                //      << "), curr: (" << curr_step_x << ", " << curr_step_y
                //      << ")"
                //      << " with taks id " << raw_steps[i][j].task_id
                //      << endl;
                Action processedAction;
                processedAction.robot_id = (int)i;
                // @jingtian Note: change action start time, to be consistent
                // with the continuous case
                processedAction.time = raw_steps[i][j - 1].time;
                processedAction.start.first = prev_step_x;
                processedAction.start.second = prev_step_y;
                processedAction.goal.first = curr_step_x;
                processedAction.goal.second = curr_step_y;
                processedAction.orientation = raw_steps[i][j].orientation;
                processedAction.nodeID = node_id;
                Action accesoray_action = processedAction;

                // Task is considered as complete when the accesoray action is
                // completed
                // accesoray_action.task_id = raw_steps[i][j].task_id;

                if (processedAction.start == processedAction.goal &&
                    raw_steps[i][j - 1].orientation !=
                        raw_steps[i][j].orientation) {
                    processedAction.type = 'T';  // Turn
                } else if (processedAction.start != processedAction.goal &&
                           raw_steps[i][j - 1].orientation ==
                               raw_steps[i][j].orientation) {
                    double mid_x = (prev_step_x + curr_step_x) / 2.0;
                    double mid_y = (prev_step_y + curr_step_y) / 2.0;
                    processedAction.type = 'M';  // Move
                    processedAction.start.first = prev_step_x;
                    processedAction.start.second = prev_step_y;
                    processedAction.goal.first = mid_x;
                    processedAction.goal.second = mid_y;
                    accesoray_action.type = 'M';
                    accesoray_action.start.first = mid_x;
                    accesoray_action.start.second = mid_y;
                    accesoray_action.goal.first = curr_step_x;
                    accesoray_action.goal.second = curr_step_y;
                    accesoray_action.nodeID = node_id + 1;
                } else if (processedAction.start == processedAction.goal &&
                           raw_steps[i][j - 1].orientation ==
                               raw_steps[i][j].orientation) {
                    continue;
                } else {
                    cerr << "Invalid case exiting ..." << endl;
                    exit(-1);
                }

                if (processedAction.type == 'T' and
                    j == (raw_steps[i].size() - 1)) {
                    continue;
                }
                processedActions.push_back(processedAction);
                if (processedAction.type == 'M') {
                    processedActions.push_back(accesoray_action);
                    node_id++;
                }
                node_id++;

                // Add special action for goal arrival
                if (raw_steps[i][j].task_id >= 0) {
                    // cout << "Adding goal action for agent " << i
                    //      << ", task_id: " << raw_steps[i][j].task_id
                    //      << endl;
                    Action goalAction(accesoray_action);
                    goalAction.type = 'S';  // Station
                    goalAction.task_id = raw_steps[i][j].task_id;
                    goalAction.nodeID = node_id;
                    // goalAction.time += 1;
                    node_id++;
                    processedActions.push_back(goalAction);
                }
            }
        }
        plans.push_back(processedActions);
    }

    if (this->screen > 1)
        showActions(plans);

    return plans;
}

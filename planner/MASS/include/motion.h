#pragma once

#include "common.h"

#define DEFAULT_V_MIN 0.0
#define DEFAULT_V_MAX 2.0
#define DEFAULT_A_MAX 2.0
#define DEFAULT_ROTATE_COST 2.0
#define DEFAULT_TURN_BACK_COST 3.6
#define DEFAULT_LENGTH 0.4  // radius of the robot

struct RobotMotion {
public:
    RobotMotion() = default;
    RobotMotion(const RobotMotion &other) {
        V_MIN = other.V_MIN;
        V_MAX = other.V_MAX;
        A_MAX = other.A_MAX;
        ROTATE_COST = other.ROTATE_COST;
        TURN_BACK_COST = other.TURN_BACK_COST;
    }

    double V_MIN = DEFAULT_V_MIN;
    double V_MAX = DEFAULT_V_MAX;
    double A_MAX = DEFAULT_A_MAX;
    double ROTATE_COST = DEFAULT_ROTATE_COST;
    double TURN_BACK_COST = DEFAULT_TURN_BACK_COST;

    // Calculates the time to accelerate from zero to max_speed or vice versa.
    inline double timeToMaxSpeed() {
        return V_MAX / A_MAX;
    }

    // Calculates the distance covered during acceleration or deceleration from
    // zero to max_speed or vice versa.
    inline int distanceDuringAcceleration() {
        double time = timeToMaxSpeed();
        // Using equation: s = ut + 0.5 * a * t^2 (where u is initial speed,
        // which is 0 here)
        return (int)(0.5 * A_MAX * time * time);
    }

    // inline double arrLowerBound(size_t step) {
    //     double total_t;
    //     size_t length = step + 2;
    //     if (step > 2 * distanceDuringAcceleration()) {
    //         total_t = 2 * timeToMaxSpeed() +
    //                   ((length - 2 * distanceDuringAcceleration() - 1) /
    //                   V_MAX);
    //     } else {
    //         total_t = 2 * sqrt((length - 1) / A_MAX);
    //     }
    //     return total_t;
    // }

    inline double arrLowerBound(size_t step) {
        double total_t;
        // size_t length = step + 2;
        if (step > 2 * distanceDuringAcceleration()) {
            total_t = 2 * timeToMaxSpeed() +
                      ((step - 2 * distanceDuringAcceleration()) / V_MAX);
        } else {
            total_t = 2 * sqrt((step) / A_MAX);
        }
        return total_t;
    }
};
#include "trajectory_loop_functions.h"

/****************************************/
/****************************************/

/*
 * To reduce the number of waypoints stored in memory,
 * consider two robot positions distinct if they are
 * at least MIN_DISTANCE away from each other
 * This constant is expressed in meters
 */
static const Real MIN_DISTANCE = 0.05f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::readPickerPath() {
std::vector<std::vector<std::pair<int, int>>> read_picker_path = {
{ { 0, 15 }, { 0, 14 }, { 0, 13 }, { 0, 12 }, { 0, 11 }, { 0, 10 }, { 0, 9 }, { 0, 8 }, { 0, 7 }, { 0, 6 }, { 0, 5 }, { 0, 4 }, { 0, 3 }, { 1, 3 }, { 2, 3 }, { 3, 3 }, { 4, 3 }, { 5, 3 }, { 6, 3 }, { 7, 3 }, { 7, 4 }, { 7, 5 }, { 7, 6 }, { 7, 7 }, { 7, 8 }, { 7, 9 }, { 7, 10 }, { 7, 11 }, { 7, 12 }, { 7, 13 }, { 7, 14 }, { 7, 15 }, { 7, 16 }, { 7, 17 }, { 7, 18 }, { 7, 19 }, { 7, 20 }, { 7, 21 }, { 7, 22 }, { 7, 23 }, { 7, 24 }, { 7, 25 }, { 7, 26 }, { 7, 27 }, { 7, 28 }, { 7, 29 }, { 7, 30 }, { 7, 31 }, { 7, 32 }, { 7, 33 }, { 7, 34 }, { 7, 35 }, { 7, 36 }, { 7, 37 }, { 7, 38 }, { 7, 39 }, { 6, 39 }, { 5, 39 }, { 4, 39 }, { 3, 39 }, { 2, 39 }, { 1, 39 }, { 0, 39 }, { 0, 38 }, { 0, 37 }, { 0, 36 }, { 0, 35 }, { 0, 34 }, { 0, 33 }, { 0, 32 }, { 0, 31 }, { 0, 30 }, { 0, 29 }, { 0, 28 }, { 0, 27 }, { 0, 26 }, { 0, 25 }, { 0, 24 }, { 0, 23 }, { 0, 22 }, { 0, 21 }, { 0, 20 }, { 0, 19 }, { 0, 18 }, { 0, 17 }, { 0, 16 }, },
{ { 3, 31 }, { 3, 30 }, { 3, 29 }, { 3, 28 }, { 3, 27 }, { 3, 26 }, { 3, 25 }, { 3, 24 }, { 3, 23 }, { 3, 22 }, { 3, 21 }, { 3, 20 }, { 3, 19 }, { 3, 18 }, { 3, 17 }, { 3, 16 }, { 3, 15 }, { 3, 14 }, { 3, 13 }, { 3, 12 }, { 3, 11 }, { 3, 10 }, { 3, 9 }, { 3, 8 }, { 3, 7 }, { 3, 6 }, { 3, 5 }, { 3, 4 }, { 4, 4 }, { 4, 5 }, { 4, 6 }, { 4, 7 }, { 4, 8 }, { 4, 9 }, { 4, 10 }, { 4, 11 }, { 4, 12 }, { 4, 13 }, { 4, 14 }, { 4, 15 }, { 4, 16 }, { 4, 17 }, { 4, 18 }, { 4, 19 }, { 4, 20 }, { 4, 21 }, { 4, 22 }, { 4, 23 }, { 4, 24 }, { 4, 25 }, { 4, 26 }, { 4, 27 }, { 4, 28 }, { 4, 29 }, { 4, 30 }, { 4, 31 }, { 4, 32 }, { 4, 33 }, { 4, 34 }, { 4, 35 }, { 4, 36 }, { 4, 37 }, { 4, 38 }, { 3, 38 }, { 3, 37 }, { 3, 36 }, { 3, 35 }, { 3, 34 }, { 3, 33 }, { 3, 32 }, },
{ { 4, 15 }, { 4, 16 }, { 4, 17 }, { 4, 18 }, { 4, 19 }, { 4, 20 }, { 4, 21 }, { 4, 22 }, { 4, 23 }, { 4, 24 }, { 4, 25 }, { 4, 26 }, { 4, 27 }, { 4, 28 }, { 4, 29 }, { 4, 30 }, { 4, 31 }, { 4, 32 }, { 4, 33 }, { 4, 34 }, { 4, 35 }, { 4, 36 }, { 4, 37 }, { 4, 38 }, { 3, 38 }, { 3, 37 }, { 3, 36 }, { 3, 35 }, { 3, 34 }, { 3, 33 }, { 3, 32 }, { 3, 31 }, { 3, 30 }, { 3, 29 }, { 3, 28 }, { 3, 27 }, { 3, 26 }, { 3, 25 }, { 3, 24 }, { 3, 23 }, { 3, 22 }, { 3, 21 }, { 3, 20 }, { 3, 19 }, { 3, 18 }, { 3, 17 }, { 3, 16 }, { 3, 15 }, { 3, 14 }, { 3, 13 }, { 3, 12 }, { 3, 11 }, { 3, 10 }, { 3, 9 }, { 3, 8 }, { 3, 7 }, { 3, 6 }, { 3, 5 }, { 3, 4 }, { 4, 4 }, { 4, 5 }, { 4, 6 }, { 4, 7 }, { 4, 8 }, { 4, 9 }, { 4, 10 }, { 4, 11 }, { 4, 12 }, { 4, 13 }, { 4, 14 }, },
{ { 7, 35 }, { 7, 36 }, { 7, 37 }, { 7, 38 }, { 7, 39 }, { 6, 39 }, { 5, 39 }, { 4, 39 }, { 3, 39 }, { 2, 39 }, { 1, 39 }, { 0, 39 }, { 0, 38 }, { 0, 37 }, { 0, 36 }, { 0, 35 }, { 0, 34 }, { 0, 33 }, { 0, 32 }, { 0, 31 }, { 0, 30 }, { 0, 29 }, { 0, 28 }, { 0, 27 }, { 0, 26 }, { 0, 25 }, { 0, 24 }, { 0, 23 }, { 0, 22 }, { 0, 21 }, { 0, 20 }, { 0, 19 }, { 0, 18 }, { 0, 17 }, { 0, 16 }, { 0, 15 }, { 0, 14 }, { 0, 13 }, { 0, 12 }, { 0, 11 }, { 0, 10 }, { 0, 9 }, { 0, 8 }, { 0, 7 }, { 0, 6 }, { 0, 5 }, { 0, 4 }, { 0, 3 }, { 1, 3 }, { 2, 3 }, { 3, 3 }, { 4, 3 }, { 5, 3 }, { 6, 3 }, { 7, 3 }, { 7, 4 }, { 7, 5 }, { 7, 6 }, { 7, 7 }, { 7, 8 }, { 7, 9 }, { 7, 10 }, { 7, 11 }, { 7, 12 }, { 7, 13 }, { 7, 14 }, { 7, 15 }, { 7, 16 }, { 7, 17 }, { 7, 18 }, { 7, 19 }, { 7, 20 }, { 7, 21 }, { 7, 22 }, { 7, 23 }, { 7, 24 }, { 7, 25 }, { 7, 26 }, { 7, 27 }, { 7, 28 }, { 7, 29 }, { 7, 30 }, { 7, 31 }, { 7, 32 }, { 7, 33 }, { 7, 34 }, },
{ { 16, 6 }, { 16, 5 }, { 16, 4 }, { 16, 3 }, { 17, 3 }, { 18, 3 }, { 19, 3 }, { 20, 3 }, { 21, 3 }, { 22, 3 }, { 23, 3 }, { 23, 4 }, { 23, 5 }, { 23, 6 }, { 23, 7 }, { 23, 8 }, { 23, 9 }, { 23, 10 }, { 23, 11 }, { 23, 12 }, { 23, 13 }, { 23, 14 }, { 23, 15 }, { 23, 16 }, { 23, 17 }, { 23, 18 }, { 23, 19 }, { 23, 20 }, { 23, 21 }, { 23, 22 }, { 23, 23 }, { 23, 24 }, { 23, 25 }, { 23, 26 }, { 23, 27 }, { 23, 28 }, { 23, 29 }, { 23, 30 }, { 23, 31 }, { 23, 32 }, { 23, 33 }, { 23, 34 }, { 23, 35 }, { 23, 36 }, { 23, 37 }, { 23, 38 }, { 23, 39 }, { 22, 39 }, { 21, 39 }, { 20, 39 }, { 19, 39 }, { 18, 39 }, { 17, 39 }, { 16, 39 }, { 16, 38 }, { 16, 37 }, { 16, 36 }, { 16, 35 }, { 16, 34 }, { 16, 33 }, { 16, 32 }, { 16, 31 }, { 16, 30 }, { 16, 29 }, { 16, 28 }, { 16, 27 }, { 16, 26 }, { 16, 25 }, { 16, 24 }, { 16, 23 }, { 16, 22 }, { 16, 21 }, { 16, 20 }, { 16, 19 }, { 16, 18 }, { 16, 17 }, { 16, 16 }, { 16, 15 }, { 16, 14 }, { 16, 13 }, { 16, 12 }, { 16, 11 }, { 16, 10 }, { 16, 9 }, { 16, 8 }, { 16, 7 }, },
{ { 19, 14 }, { 19, 13 }, { 19, 12 }, { 19, 11 }, { 19, 10 }, { 19, 9 }, { 19, 8 }, { 19, 7 }, { 19, 6 }, { 19, 5 }, { 19, 4 }, { 20, 4 }, { 20, 5 }, { 20, 6 }, { 20, 7 }, { 20, 8 }, { 20, 9 }, { 20, 10 }, { 20, 11 }, { 20, 12 }, { 20, 13 }, { 20, 14 }, { 20, 15 }, { 20, 16 }, { 20, 17 }, { 20, 18 }, { 20, 19 }, { 20, 20 }, { 20, 21 }, { 20, 22 }, { 20, 23 }, { 20, 24 }, { 20, 25 }, { 20, 26 }, { 20, 27 }, { 20, 28 }, { 20, 29 }, { 20, 30 }, { 20, 31 }, { 20, 32 }, { 20, 33 }, { 20, 34 }, { 20, 35 }, { 20, 36 }, { 20, 37 }, { 20, 38 }, { 19, 38 }, { 19, 37 }, { 19, 36 }, { 19, 35 }, { 19, 34 }, { 19, 33 }, { 19, 32 }, { 19, 31 }, { 19, 30 }, { 19, 29 }, { 19, 28 }, { 19, 27 }, { 19, 26 }, { 19, 25 }, { 19, 24 }, { 19, 23 }, { 19, 22 }, { 19, 21 }, { 19, 20 }, { 19, 19 }, { 19, 18 }, { 19, 17 }, { 19, 16 }, { 19, 15 }, },
{ { 20, 12 }, { 20, 13 }, { 20, 14 }, { 20, 15 }, { 20, 16 }, { 20, 17 }, { 20, 18 }, { 20, 19 }, { 20, 20 }, { 20, 21 }, { 20, 22 }, { 20, 23 }, { 20, 24 }, { 20, 25 }, { 20, 26 }, { 20, 27 }, { 20, 28 }, { 20, 29 }, { 20, 30 }, { 20, 31 }, { 20, 32 }, { 20, 33 }, { 20, 34 }, { 20, 35 }, { 20, 36 }, { 20, 37 }, { 20, 38 }, { 19, 38 }, { 19, 37 }, { 19, 36 }, { 19, 35 }, { 19, 34 }, { 19, 33 }, { 19, 32 }, { 19, 31 }, { 19, 30 }, { 19, 29 }, { 19, 28 }, { 19, 27 }, { 19, 26 }, { 19, 25 }, { 19, 24 }, { 19, 23 }, { 19, 22 }, { 19, 21 }, { 19, 20 }, { 19, 19 }, { 19, 18 }, { 19, 17 }, { 19, 16 }, { 19, 15 }, { 19, 14 }, { 19, 13 }, { 19, 12 }, { 19, 11 }, { 19, 10 }, { 19, 9 }, { 19, 8 }, { 19, 7 }, { 19, 6 }, { 19, 5 }, { 19, 4 }, { 20, 4 }, { 20, 5 }, { 20, 6 }, { 20, 7 }, { 20, 8 }, { 20, 9 }, { 20, 10 }, { 20, 11 }, },
{ { 23, 31 }, { 23, 32 }, { 23, 33 }, { 23, 34 }, { 23, 35 }, { 23, 36 }, { 23, 37 }, { 23, 38 }, { 23, 39 }, { 22, 39 }, { 21, 39 }, { 20, 39 }, { 19, 39 }, { 18, 39 }, { 17, 39 }, { 16, 39 }, { 16, 38 }, { 16, 37 }, { 16, 36 }, { 16, 35 }, { 16, 34 }, { 16, 33 }, { 16, 32 }, { 16, 31 }, { 16, 30 }, { 16, 29 }, { 16, 28 }, { 16, 27 }, { 16, 26 }, { 16, 25 }, { 16, 24 }, { 16, 23 }, { 16, 22 }, { 16, 21 }, { 16, 20 }, { 16, 19 }, { 16, 18 }, { 16, 17 }, { 16, 16 }, { 16, 15 }, { 16, 14 }, { 16, 13 }, { 16, 12 }, { 16, 11 }, { 16, 10 }, { 16, 9 }, { 16, 8 }, { 16, 7 }, { 16, 6 }, { 16, 5 }, { 16, 4 }, { 16, 3 }, { 17, 3 }, { 18, 3 }, { 19, 3 }, { 20, 3 }, { 21, 3 }, { 22, 3 }, { 23, 3 }, { 23, 4 }, { 23, 5 }, { 23, 6 }, { 23, 7 }, { 23, 8 }, { 23, 9 }, { 23, 10 }, { 23, 11 }, { 23, 12 }, { 23, 13 }, { 23, 14 }, { 23, 15 }, { 23, 16 }, { 23, 17 }, { 23, 18 }, { 23, 19 }, { 23, 20 }, { 23, 21 }, { 23, 22 }, { 23, 23 }, { 23, 24 }, { 23, 25 }, { 23, 26 }, { 23, 27 }, { 23, 28 }, { 23, 29 }, { 23, 30 }, },
{ { 24, 13 }, { 24, 12 }, { 24, 11 }, { 24, 10 }, { 24, 9 }, { 24, 8 }, { 24, 7 }, { 24, 6 }, { 24, 5 }, { 24, 4 }, { 24, 3 }, { 25, 3 }, { 26, 3 }, { 27, 3 }, { 28, 3 }, { 29, 3 }, { 30, 3 }, { 31, 3 }, { 31, 4 }, { 31, 5 }, { 31, 6 }, { 31, 7 }, { 31, 8 }, { 31, 9 }, { 31, 10 }, { 31, 11 }, { 31, 12 }, { 31, 13 }, { 31, 14 }, { 31, 15 }, { 31, 16 }, { 31, 17 }, { 31, 18 }, { 31, 19 }, { 31, 20 }, { 31, 21 }, { 31, 22 }, { 31, 23 }, { 31, 24 }, { 31, 25 }, { 31, 26 }, { 31, 27 }, { 31, 28 }, { 31, 29 }, { 31, 30 }, { 31, 31 }, { 31, 32 }, { 31, 33 }, { 31, 34 }, { 31, 35 }, { 31, 36 }, { 31, 37 }, { 31, 38 }, { 31, 39 }, { 30, 39 }, { 29, 39 }, { 28, 39 }, { 27, 39 }, { 26, 39 }, { 25, 39 }, { 24, 39 }, { 24, 38 }, { 24, 37 }, { 24, 36 }, { 24, 35 }, { 24, 34 }, { 24, 33 }, { 24, 32 }, { 24, 31 }, { 24, 30 }, { 24, 29 }, { 24, 28 }, { 24, 27 }, { 24, 26 }, { 24, 25 }, { 24, 24 }, { 24, 23 }, { 24, 22 }, { 24, 21 }, { 24, 20 }, { 24, 19 }, { 24, 18 }, { 24, 17 }, { 24, 16 }, { 24, 15 }, { 24, 14 }, },
{ { 27, 37 }, { 27, 36 }, { 27, 35 }, { 27, 34 }, { 27, 33 }, { 27, 32 }, { 27, 31 }, { 27, 30 }, { 27, 29 }, { 27, 28 }, { 27, 27 }, { 27, 26 }, { 27, 25 }, { 27, 24 }, { 27, 23 }, { 27, 22 }, { 27, 21 }, { 27, 20 }, { 27, 19 }, { 27, 18 }, { 27, 17 }, { 27, 16 }, { 27, 15 }, { 27, 14 }, { 27, 13 }, { 27, 12 }, { 27, 11 }, { 27, 10 }, { 27, 9 }, { 27, 8 }, { 27, 7 }, { 27, 6 }, { 27, 5 }, { 27, 4 }, { 28, 4 }, { 28, 5 }, { 28, 6 }, { 28, 7 }, { 28, 8 }, { 28, 9 }, { 28, 10 }, { 28, 11 }, { 28, 12 }, { 28, 13 }, { 28, 14 }, { 28, 15 }, { 28, 16 }, { 28, 17 }, { 28, 18 }, { 28, 19 }, { 28, 20 }, { 28, 21 }, { 28, 22 }, { 28, 23 }, { 28, 24 }, { 28, 25 }, { 28, 26 }, { 28, 27 }, { 28, 28 }, { 28, 29 }, { 28, 30 }, { 28, 31 }, { 28, 32 }, { 28, 33 }, { 28, 34 }, { 28, 35 }, { 28, 36 }, { 28, 37 }, { 28, 38 }, { 27, 38 }, },
{ { 28, 17 }, { 28, 18 }, { 28, 19 }, { 28, 20 }, { 28, 21 }, { 28, 22 }, { 28, 23 }, { 28, 24 }, { 28, 25 }, { 28, 26 }, { 28, 27 }, { 28, 28 }, { 28, 29 }, { 28, 30 }, { 28, 31 }, { 28, 32 }, { 28, 33 }, { 28, 34 }, { 28, 35 }, { 28, 36 }, { 28, 37 }, { 28, 38 }, { 27, 38 }, { 27, 37 }, { 27, 36 }, { 27, 35 }, { 27, 34 }, { 27, 33 }, { 27, 32 }, { 27, 31 }, { 27, 30 }, { 27, 29 }, { 27, 28 }, { 27, 27 }, { 27, 26 }, { 27, 25 }, { 27, 24 }, { 27, 23 }, { 27, 22 }, { 27, 21 }, { 27, 20 }, { 27, 19 }, { 27, 18 }, { 27, 17 }, { 27, 16 }, { 27, 15 }, { 27, 14 }, { 27, 13 }, { 27, 12 }, { 27, 11 }, { 27, 10 }, { 27, 9 }, { 27, 8 }, { 27, 7 }, { 27, 6 }, { 27, 5 }, { 27, 4 }, { 28, 4 }, { 28, 5 }, { 28, 6 }, { 28, 7 }, { 28, 8 }, { 28, 9 }, { 28, 10 }, { 28, 11 }, { 28, 12 }, { 28, 13 }, { 28, 14 }, { 28, 15 }, { 28, 16 }, },
{ { 31, 31 }, { 31, 32 }, { 31, 33 }, { 31, 34 }, { 31, 35 }, { 31, 36 }, { 31, 37 }, { 31, 38 }, { 31, 39 }, { 30, 39 }, { 29, 39 }, { 28, 39 }, { 27, 39 }, { 26, 39 }, { 25, 39 }, { 24, 39 }, { 24, 38 }, { 24, 37 }, { 24, 36 }, { 24, 35 }, { 24, 34 }, { 24, 33 }, { 24, 32 }, { 24, 31 }, { 24, 30 }, { 24, 29 }, { 24, 28 }, { 24, 27 }, { 24, 26 }, { 24, 25 }, { 24, 24 }, { 24, 23 }, { 24, 22 }, { 24, 21 }, { 24, 20 }, { 24, 19 }, { 24, 18 }, { 24, 17 }, { 24, 16 }, { 24, 15 }, { 24, 14 }, { 24, 13 }, { 24, 12 }, { 24, 11 }, { 24, 10 }, { 24, 9 }, { 24, 8 }, { 24, 7 }, { 24, 6 }, { 24, 5 }, { 24, 4 }, { 24, 3 }, { 25, 3 }, { 26, 3 }, { 27, 3 }, { 28, 3 }, { 29, 3 }, { 30, 3 }, { 31, 3 }, { 31, 4 }, { 31, 5 }, { 31, 6 }, { 31, 7 }, { 31, 8 }, { 31, 9 }, { 31, 10 }, { 31, 11 }, { 31, 12 }, { 31, 13 }, { 31, 14 }, { 31, 15 }, { 31, 16 }, { 31, 17 }, { 31, 18 }, { 31, 19 }, { 31, 20 }, { 31, 21 }, { 31, 22 }, { 31, 23 }, { 31, 24 }, { 31, 25 }, { 31, 26 }, { 31, 27 }, { 31, 28 }, { 31, 29 }, { 31, 30 }, },
{ { 32, 14 }, { 32, 13 }, { 32, 12 }, { 32, 11 }, { 32, 10 }, { 32, 9 }, { 32, 8 }, { 32, 7 }, { 32, 6 }, { 32, 5 }, { 32, 4 }, { 32, 3 }, { 33, 3 }, { 34, 3 }, { 35, 3 }, { 36, 3 }, { 37, 3 }, { 38, 3 }, { 39, 3 }, { 39, 4 }, { 39, 5 }, { 39, 6 }, { 39, 7 }, { 39, 8 }, { 39, 9 }, { 39, 10 }, { 39, 11 }, { 39, 12 }, { 39, 13 }, { 39, 14 }, { 39, 15 }, { 39, 16 }, { 39, 17 }, { 39, 18 }, { 39, 19 }, { 39, 20 }, { 39, 21 }, { 39, 22 }, { 39, 23 }, { 39, 24 }, { 39, 25 }, { 39, 26 }, { 39, 27 }, { 39, 28 }, { 39, 29 }, { 39, 30 }, { 39, 31 }, { 39, 32 }, { 39, 33 }, { 39, 34 }, { 39, 35 }, { 39, 36 }, { 39, 37 }, { 39, 38 }, { 39, 39 }, { 38, 39 }, { 37, 39 }, { 36, 39 }, { 35, 39 }, { 34, 39 }, { 33, 39 }, { 32, 39 }, { 32, 38 }, { 32, 37 }, { 32, 36 }, { 32, 35 }, { 32, 34 }, { 32, 33 }, { 32, 32 }, { 32, 31 }, { 32, 30 }, { 32, 29 }, { 32, 28 }, { 32, 27 }, { 32, 26 }, { 32, 25 }, { 32, 24 }, { 32, 23 }, { 32, 22 }, { 32, 21 }, { 32, 20 }, { 32, 19 }, { 32, 18 }, { 32, 17 }, { 32, 16 }, { 32, 15 }, },
{ { 35, 37 }, { 35, 36 }, { 35, 35 }, { 35, 34 }, { 35, 33 }, { 35, 32 }, { 35, 31 }, { 35, 30 }, { 35, 29 }, { 35, 28 }, { 35, 27 }, { 35, 26 }, { 35, 25 }, { 35, 24 }, { 35, 23 }, { 35, 22 }, { 35, 21 }, { 35, 20 }, { 35, 19 }, { 35, 18 }, { 35, 17 }, { 35, 16 }, { 35, 15 }, { 35, 14 }, { 35, 13 }, { 35, 12 }, { 35, 11 }, { 35, 10 }, { 35, 9 }, { 35, 8 }, { 35, 7 }, { 35, 6 }, { 35, 5 }, { 35, 4 }, { 36, 4 }, { 36, 5 }, { 36, 6 }, { 36, 7 }, { 36, 8 }, { 36, 9 }, { 36, 10 }, { 36, 11 }, { 36, 12 }, { 36, 13 }, { 36, 14 }, { 36, 15 }, { 36, 16 }, { 36, 17 }, { 36, 18 }, { 36, 19 }, { 36, 20 }, { 36, 21 }, { 36, 22 }, { 36, 23 }, { 36, 24 }, { 36, 25 }, { 36, 26 }, { 36, 27 }, { 36, 28 }, { 36, 29 }, { 36, 30 }, { 36, 31 }, { 36, 32 }, { 36, 33 }, { 36, 34 }, { 36, 35 }, { 36, 36 }, { 36, 37 }, { 36, 38 }, { 35, 38 }, },
{ { 36, 5 }, { 36, 6 }, { 36, 7 }, { 36, 8 }, { 36, 9 }, { 36, 10 }, { 36, 11 }, { 36, 12 }, { 36, 13 }, { 36, 14 }, { 36, 15 }, { 36, 16 }, { 36, 17 }, { 36, 18 }, { 36, 19 }, { 36, 20 }, { 36, 21 }, { 36, 22 }, { 36, 23 }, { 36, 24 }, { 36, 25 }, { 36, 26 }, { 36, 27 }, { 36, 28 }, { 36, 29 }, { 36, 30 }, { 36, 31 }, { 36, 32 }, { 36, 33 }, { 36, 34 }, { 36, 35 }, { 36, 36 }, { 36, 37 }, { 36, 38 }, { 35, 38 }, { 35, 37 }, { 35, 36 }, { 35, 35 }, { 35, 34 }, { 35, 33 }, { 35, 32 }, { 35, 31 }, { 35, 30 }, { 35, 29 }, { 35, 28 }, { 35, 27 }, { 35, 26 }, { 35, 25 }, { 35, 24 }, { 35, 23 }, { 35, 22 }, { 35, 21 }, { 35, 20 }, { 35, 19 }, { 35, 18 }, { 35, 17 }, { 35, 16 }, { 35, 15 }, { 35, 14 }, { 35, 13 }, { 35, 12 }, { 35, 11 }, { 35, 10 }, { 35, 9 }, { 35, 8 }, { 35, 7 }, { 35, 6 }, { 35, 5 }, { 35, 4 }, { 36, 4 }, },
{ { 39, 9 }, { 39, 10 }, { 39, 11 }, { 39, 12 }, { 39, 13 }, { 39, 14 }, { 39, 15 }, { 39, 16 }, { 39, 17 }, { 39, 18 }, { 39, 19 }, { 39, 20 }, { 39, 21 }, { 39, 22 }, { 39, 23 }, { 39, 24 }, { 39, 25 }, { 39, 26 }, { 39, 27 }, { 39, 28 }, { 39, 29 }, { 39, 30 }, { 39, 31 }, { 39, 32 }, { 39, 33 }, { 39, 34 }, { 39, 35 }, { 39, 36 }, { 39, 37 }, { 39, 38 }, { 39, 39 }, { 38, 39 }, { 37, 39 }, { 36, 39 }, { 35, 39 }, { 34, 39 }, { 33, 39 }, { 32, 39 }, { 32, 38 }, { 32, 37 }, { 32, 36 }, { 32, 35 }, { 32, 34 }, { 32, 33 }, { 32, 32 }, { 32, 31 }, { 32, 30 }, { 32, 29 }, { 32, 28 }, { 32, 27 }, { 32, 26 }, { 32, 25 }, { 32, 24 }, { 32, 23 }, { 32, 22 }, { 32, 21 }, { 32, 20 }, { 32, 19 }, { 32, 18 }, { 32, 17 }, { 32, 16 }, { 32, 15 }, { 32, 14 }, { 32, 13 }, { 32, 12 }, { 32, 11 }, { 32, 10 }, { 32, 9 }, { 32, 8 }, { 32, 7 }, { 32, 6 }, { 32, 5 }, { 32, 4 }, { 32, 3 }, { 33, 3 }, { 34, 3 }, { 35, 3 }, { 36, 3 }, { 37, 3 }, { 38, 3 }, { 39, 3 }, { 39, 4 }, { 39, 5 }, { 39, 6 }, { 39, 7 }, { 39, 8 }, },
};
  all_picker_paths = read_picker_path;
}

void CTrajectoryLoopFunctions::Init(TConfigurationNode& t_tree) {
  /*
  * Go through all the robots in the environment
  * and create an entry in the waypoint map for each of them
  */
  /* Get the map of all foot-bots from the space */
  CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
  /* Go through them */
  for(CSpace::TMapPerType::iterator it = tFBMap.begin();
      it != tFBMap.end();
      ++it) {
    /* Create a pointer to the current foot-bot */
    CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
    /* Create a waypoint vector */
    m_tWaypoints[pcFB] = std::vector<CVector3>();
    /* Add the initial position of the foot-bot */
    m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
  }
  int num_station = 0;
  TConfigurationNode& tParams = GetNode(t_tree, "num_stations");
  GetNodeAttribute(tParams, "value", num_station);
  TConfigurationNode& portParams = GetNode(t_tree, "port_number");
  GetNodeAttribute(portParams, "value", port_number);
  // TConfigurationNode& pickerParams = GetNode(t_tree, "num_pickers");
  // GetNodeAttribute(pickerParams, "value", num_picker);
  for (int i = 0; i < num_station; i++) {
    Real x, y, z;
    TConfigurationNode& tStation = GetNode(t_tree, "station" + std::to_string(i));
    GetNodeAttribute(tStation, "x", x);
    GetNodeAttribute(tStation, "y", y);
    GetNodeAttribute(tStation, "z", z);
    all_stations.emplace_back(x, y, z);
    // std::cout << "Station: " << x << ", " << y << ", " << z << std::endl;
  }
  readPickerPath();
  num_picker = static_cast<int>(all_picker_paths.size());
  all_pickers.resize(num_picker);
//   task_goals.emplace_back(-3.0, -0.0, 0.0);
//   task_goals.emplace_back(-0.0, -3.0, 0.0);
}

/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::Reset() {
   /*
    * Clear all the waypoint vectors
    */
   /* Get the map of all foot-bots from the space */
   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      /* Clear the waypoint vector */
      m_tWaypoints[pcFB].clear();
      /* Add the initial position of the foot-bot */
      m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
   }
}

/****************************************/

/**
 * Request for a new mobile robot to pick up the order
 *
 * @return Task id for the request
 */
int CTrajectoryLoopFunctions::requestMobileRobot(std::pair<int, int>& loc) {
  int new_mobile_task_id = client->call("request_mobile_robot", std::make_pair(loc.second, loc.first)).as<int>();
  return new_mobile_task_id;
}

/**
 * Get the actions at the next location for the agent
 *
 * @param agent_id id of the agent
 */
void CTrajectoryLoopFunctions::getNextAction(int agent_id) {
  printf("Get next action for agent %d\n", agent_id);
  auto& curr_agent = all_pickers[agent_id];
  auto& last_act = curr_agent.acts.back();
  assert(last_act.act == MOVE);
  auto pick_task_it = all_pick_tasks.find(last_act.end);
  if (pick_task_it != all_pick_tasks.end()) {
    for (auto tmp_task_id : pick_task_it->second) {
      curr_agent.acts.emplace_back(PICK_T, PICK, pick_task_it->first, pick_task_it->first, -1, tmp_task_id);
      curr_agent.curr_load += 1;
      printf("Find pickup task with id %d, current load is %d\n", tmp_task_id, curr_agent.curr_load);
      if (curr_agent.curr_load >= LOAD_NUM) {
        // OK@jingtian: need to change this to communicate
        int new_mobile_task_id = requestMobileRobot(last_act.end);
        printf("Add new mobile request task for agent %d with id %d\n", agent_id, new_mobile_task_id);
        curr_agent.acts.emplace_back(UNLOAD_T, UNLOAD, pick_task_it->first, pick_task_it->first, -1, new_mobile_task_id);
        curr_agent.curr_load = 0;
      }
    }
    // remove the all pickup task, since they are finished
    all_pick_tasks.erase(pick_task_it);
  }
  std::pair<int, int> next_loc;
  int next_step_id = nextloc(agent_id, last_act.step_id, next_loc);
  curr_agent.acts.emplace_back(MOVE_T, MOVE, last_act.end, next_loc, next_step_id);
}

/**
 * Get the actions at the next location for the agent
 *
 * @param agent_id id of the agent
 */
void CTrajectoryLoopFunctions::initRobot(int agent_id) {
  printf("Get first action for agent %d\n", agent_id);
  auto& curr_agent = all_pickers[agent_id];
  curr_agent.acts.emplace_back(MOVE_T, MOVE, all_picker_paths[agent_id].front(), all_picker_paths[agent_id].front(), 0);
  printf("Finish first action for agent %d\n", agent_id);
}

/**
 * Change the coordinates from 2-D map to simulation
 *
 * @param loc 2-D coord from planner
 * @return location in 3-D simulation
 */
inline CVector3 CTrajectoryLoopFunctions::coordPlanner2Sim(std::pair<int, int>& loc) {
  int x = loc.first;
  int y = loc.second;
  double sim_x, sim_y;
  if (x == 0) {
    sim_y = 0.0;
  } else {
    sim_y = (double) -x;
  }

  if (y == 0) {
    sim_x = 0.0;
  } else {
    sim_x = (double) -y;
  }
  return CVector3(sim_x, sim_y, 0.0);
}

/**
 * Change the coordinates from 3-D simulation to 2-D map
 *
 * @param loc 3-D coord from simulation
 * @return coords in 2-D map
 */
inline std::pair<int, int> CTrajectoryLoopFunctions::coordSim2Planner(CVector3& loc) {
  double x = loc.GetX();
  double y = loc.GetY();
  int map_x, map_y;
  if (x == 0) {
    map_y = 0;
  } else {
    map_y = static_cast<int>(-x);
  }

  if (y == 0) {
    map_x = 0;
  } else {
    map_x = static_cast<int>(-y);
  }
  return std::make_pair(map_x, map_y);
}

/**
 * Execute the move action
 *
 * @param agent_id  idx of the agent
 * @param act       current action being executed
 * @return return true if action finished, return false otherwise
 **/
bool CTrajectoryLoopFunctions::executeMove(int agent_id, PickerAction& act) {
  act.timer--;
  CVector3 picker_start = coordPlanner2Sim(act.start);
  CVector3 picker_end = coordPlanner2Sim(act.end);
  double curr_x = static_cast<double>(act.timer)/static_cast<double>(MOVE_T) * picker_start.GetX() +
    static_cast<double>(MOVE_T - act.timer)/static_cast<double>(MOVE_T) * picker_end.GetX();
  double curr_y = static_cast<double>(act.timer)/static_cast<double>(MOVE_T) * picker_start.GetY() +
    static_cast<double>(MOVE_T - act.timer)/static_cast<double>(MOVE_T) * picker_end.GetY();
  CVector3 curr_picker_curr = CVector3(curr_x, curr_y, picker_start.GetZ());
  picker_curr_locs.emplace_back(curr_picker_curr);
  if (act.timer == 0) {
    return true;
  }
  return false;
}

/**
 * Execute the pickup task for picker
 *
 * @param agent_id
 * @param act
 * @return return true if action finished, return false otherwise
 */
bool CTrajectoryLoopFunctions::executePick(int agent_id, PickerAction& act) {
  act.timer--;
  CVector3 curr_picker_obj = coordPlanner2Sim(act.start);
  curr_picking_objs.emplace_back(curr_picker_obj);
  if (act.timer == 0) {
    return true;
  }
  return false;
}

/**
 * Execute the task that unload the objects to the transport robot
 *
 * @param agent_id
 * @param act
 * @return return true if action finished, return false otherwise
 */
bool CTrajectoryLoopFunctions::executeUnload(int agent_id, PickerAction& act) {
  /*
   * OK@jingtian
   * Step 1: set the vector to indicate picker is arrived
   * Step 2: check if the mobile robot is waiting
   * Step 3:
   *    - If the mobile robot has arrived, start unload
   *    - Otherwise, wait and do nothing
   */
  // throw std::logic_error("Not implemented");
  int act_task_id = act.task_id;

  bool task_assigned = false;
  std::map<int, std::pair<bool, bool>>::iterator task_status_it;
  // printf("Executing unload action with id %d... Search task in mobile robot's queue\n", act_task_id);
  /* Get the map of all foot-bots from the space */
  CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
  for(CSpace::TMapPerType::iterator it = tFBMap.begin(); it != tFBMap.end(); ++it) {
    /* Create a pointer to the current foot-bot */
    CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
    // Get the controller
    CFootBotDiffusion* pcController = dynamic_cast<CFootBotDiffusion*>(
        &(pcFB->GetControllableEntity().GetController())
    );
    if (pcController) {
      // for (auto tmp_mobile_picker_task: pcController->picker_task) {
      //   printf("Picker task with id %d, status of (%d, %d)\n", tmp_mobile_picker_task.first,
      //     tmp_mobile_picker_task.second.first, tmp_mobile_picker_task.second.second);
      // }
      auto tmp_task_it = pcController->picker_task.find(act_task_id);
      if (tmp_task_it != pcController->picker_task.end()) {
        printf("Find task in mobile robot's queue\n");
        task_assigned = true;
        task_status_it = tmp_task_it;
        break;
      }
    }
  }
  CVector3 picker_unload = coordPlanner2Sim(act.start);
  picker_unload_locs.emplace_back(picker_unload);

  if (task_assigned) {
    if (not task_status_it->second.second) {
      task_status_it->second.second = true;
    }
    printf("Task status is: (%d, %d)\n", task_status_it->second.first, task_status_it->second.second);

    if (task_status_it->second.first) {
      act.timer--;
      // CVector3 picker_unload = coordPlanner2Sim(act.start);
      // picker_unload_locs.emplace_back(picker_unload);
      if (act.timer == 0) {
        return true;
      }
    }
  }
  return false;
}

typedef std::tuple<int, int, int> PickData;

void CTrajectoryLoopFunctions::requestNewPickTasks() {
  all_pick_tasks.clear();
  std::vector< PickData > picker_data = client->call("get_picker_task").as< std::vector<PickData> >();
  printf("Get picker tasks\n");
  for (auto& tmp_task: picker_data) {
    std::pair<int, int> task_loc = std::make_pair(std::get<1> (tmp_task), std::get<0> (tmp_task));
    printf("Pick task at (%d, %d), task id: %d\n", task_loc.first, task_loc.second, std::get<2> (tmp_task));
    auto entry = all_pick_tasks.find(task_loc);
    if (entry != all_pick_tasks.end()) {
      entry->second.push_back(std::get<2> (tmp_task));
    } else {
      all_pick_tasks[task_loc] = std::deque<int>{std::get<2> (tmp_task)};
    }
  }
  printf("Finish request new tasks\n");
}

void CTrajectoryLoopFunctions::initActionQueue() {
  requestNewPickTasks();
  // Insert initial actions
  // for (int i = 0; i < num_picker; i++) {
  //   initRobot(i);
  //   for (int j = 0; j < WINDOW_SIE; j++) {
  //     getNextAction(i);
  //   }
  // }
  for (int j = 0; j < WINDOW_SIE; j++) {
    for (int i = 0; i < num_picker; i++) {
      if (j == 0) {
        initRobot(i);
      }
      getNextAction(i);
    }
  }
}

void CTrajectoryLoopFunctions::addMobileVisualization() {

  /* Get the map of all foot-bots from the space */
  CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
  /* Go through them */
  task_pods.clear();
  task_stations.clear();
  for (CSpace::TMapPerType::iterator it = tFBMap.begin(); it != tFBMap.end(); ++it) {
    /* Create a pointer to the current foot-bot */
    CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);

    // Get the controller
    CFootBotDiffusion* pcController = dynamic_cast<CFootBotDiffusion*>(
        &(pcFB->GetControllableEntity().GetController())
    );

    if (pcController) {
      // Retrieve step count or any other return value
      // LOG << "Step Count: " << pcController->getCurrGoal() << std::endl;
      auto tmp_goal = pcController->getCurrPod();
      if (tmp_goal.GetZ() != -100) {
        task_pods.push_back(tmp_goal);
      }
      auto tmp_station = pcController->getCurrStation();
      if (tmp_station.GetZ() != -100) {
        task_stations.push_back(tmp_station);
      }
    }
  }
}
/****************************************/

void CTrajectoryLoopFunctions::PostStep() {
  // std::cout << "Try to connect to server with port num: " << port_number << std::endl;
  if (not is_initialized) {
    if (is_port_open("127.0.0.1", port_number)) {
      client = std::make_shared<rpc::client>("127.0.0.1", port_number);
    } else {
      // std::cout << "Failed to connect to server. Retrying..." << std::endl;
      // std::this_thread::sleep_for(std::chrono::seconds(5)); // Wait for 1 second before retrying
      return;
    }
    initActionQueue();
    is_initialized = true;
  }

  picker_curr_locs.clear();
  curr_picking_objs.clear();
  picker_unload_locs.clear();

  // printf("start execution!\n");
  for (int agent_id = 0; agent_id < num_picker; agent_id++) {
    auto& curr_picker = all_pickers[agent_id];
    auto& front_act = curr_picker.acts.front();
    // std::cout << "For agent " << agent_id << ", the action type is: " << front_act.act << ", curr time: " << front_act.timer;
    // printf("For agent %d, the action type is: %d, curr time: %d\n", agent_id, front_act.act, front_act.timer);
    bool act_status = false;
    assert(front_act.timer > 0);
    if (front_act.act == MOVE) {
      act_status = executeMove(agent_id, front_act);
    } else if (front_act.act == PICK) {
      act_status = executePick(agent_id, front_act);
    } else if (front_act.act == UNLOAD) {
      act_status = executeUnload(agent_id, front_act);
    } else {
      std::cerr << "Unrecognized action type! Exiting" << std::endl;
      exit(-1);
    }

    if (act_status) {
      if (front_act.act == MOVE) {
        getNextAction(agent_id);
      } else if (front_act.act == PICK) {
        client->call("confirm_picker_task", agent_id, front_act.task_id);
      }
      curr_picker.acts.pop_front();
    }
  }

  addMobileVisualization();
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CTrajectoryLoopFunctions, "trajectory_loop_functions")

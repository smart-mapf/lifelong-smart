#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <cmath>
#include <iostream>

#include "common.h"
#include "instance.h"

#define DEBUG_CRISE 0



class CRISE_Solver {
public:
    CRISE_Solver(Agent& sipp_curr_agent, std::deque<std::shared_ptr<IntervalEntry>>& sipp_result_nodes, orient& curr_o);
    bool solve(shared_ptr<MotionNode> &solution);

private:
    std::vector<double> accTimes(int length)
    {
        std::vector<double> acc_vec;
        double total_t = 2*sqrt((length-1)/A_MAX);
//        printf("Total runtime is: %f\n", total_t);
        for (int i = 0; i < length; i++) {
            double t_arr = 0.0;
            if (i < length / 2.0) {
                t_arr = sqrt(2.0*i/A_MAX);
                acc_vec.push_back(t_arr);
//                printf("val at %d is: %f\n", i, t_arr);
            } else {
                t_arr = total_t - acc_vec[length - 1 - i];
                acc_vec.push_back(t_arr);
//                printf("val at %d -> %d is: %f\n", i, length - i, t_arr);
            }
        }
        return acc_vec;
    }

    std::vector<double> arrTimes(int length)
    {
        std::vector<double> arr_vec;
        double total_t = -1;
        assert( length>=1 );
        if (length > 2*distanceDuringAcceleration()) {
//            printf("Can reach max speed!\n");
            std::vector<double> acc_vec = accTimes(2*distanceDuringAcceleration()+1);
            double t_arr = 0.0;
            total_t = 2*timeToMaxSpeed() + ((length - 2*distanceDuringAcceleration()-1)/V_MAX);
            for (int i = 0; i < length; i++) {
                if (i <= distanceDuringAcceleration()){
                    t_arr = acc_vec[i];
//                    printf("Acc at %d is: %f\n", i, t_arr);
                } else if ((length-i) <= distanceDuringAcceleration()){
                    t_arr = total_t - acc_vec[length - 1 - i];
//                    printf("Dec at %d is: %f\n", i, t_arr);
                } else {
                    t_arr = timeToMaxSpeed()+(i - distanceDuringAcceleration())/V_MAX;
//                    printf("Moving at %d is: %f\n", i, t_arr);
                }
                arr_vec.push_back(t_arr);
            }
        } else {
//            printf("Only acc and dec!\n");
            total_t = 2*sqrt((length-1)/A_MAX);
            arr_vec = accTimes(length);
        }
        optimal_T = total_t;
        return arr_vec;
    }
    /**
     *
     * @param step The step of entries on the path
     * @return
     */
    std::vector<std::pair<double, double>> occupancyTimes(int length);

private:
    Agent& curr_agent;
    IntervalSeq result_nodes;
    orient curr_o;
    double optimal_T;
    bool interval_too_small = false;
    int n_points = CONTROL_POINTS_NUM;
    double total_runtime_ = 0.0;
};
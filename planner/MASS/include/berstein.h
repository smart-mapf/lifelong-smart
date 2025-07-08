/**
 * This file is the implementation of the Bernstein Polynomial
 * 
*/
#pragma once
#include <iostream>
#include <vector>
#include <math.h>

/**
 * Store the derivative information of the Bernstein Polynomial
*/
struct BernDeriv{
    int N;
    int deriv_order;
    double ratio;

    std::vector<std::vector<int>> orignal_idx;
    std::vector<std::vector<int>> sign_idx;
    // Ratio of N/T
};

/**
 * This class serve as the base of the bezier curve
*/
class BernsteinPolynomial{
public:
    BernsteinPolynomial(int user_control_points, double time_duration);
    BernsteinPolynomial(int user_control_points, double time_duration, BernDeriv curr_deriv);
    long double ChooseNFromR(int n, int r); 
    double Berstein(int idx, double t);
    bool CalculateVal(double t, std::vector<double>& val_vec);
    BernsteinPolynomial Derivative();
    double GetVal(double t, std::vector<double>& control_points);

    BernDeriv control_points_deriv;

private:
    int N;
    double T;
};

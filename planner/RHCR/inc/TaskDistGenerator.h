#pragma once

#include <vector>
#include <cmath>
#include <random>
#include <iostream>

struct DistributionParams{
    double sigma = 1.0;
    int K = 3;
};
extern DistributionParams dist_params;

std::vector<std::vector<double>> getGaussian(int h, int w, int center_h, int center_w, double sigma = dist_params.sigma);
std::vector<double> generateVecEDist(const std::vector<std::vector<double>>& map_e, const std::vector<std::vector<double>>& dist_e);
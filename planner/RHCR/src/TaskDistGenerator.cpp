#include "TaskDistGenerator.h"


using namespace std;

DistributionParams dist_params = DistributionParams();

// Function to generate a Gaussian distribution
vector<vector<double>> getGaussian(int h, int w, int center_h, int center_w, double sigma) {
    std::cout << "sigma = " << sigma <<std::endl;
    vector<double> x0(w);
    vector<double> y0(h);
    for (int i = 0; i < w; ++i) x0[i] = -5 + i * 10.0 / (w - 1);
    for (int i = 0; i < h; ++i) y0[i] = -5 + i * 10.0 / (h - 1);

    vector<vector<double>> z(h, vector<double>(w, 0));
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            double x_diff = (x0[c] - x0[center_w]) * (x0[c] - x0[center_w]);
            double y_diff = (y0[r] - y0[center_h]) * (y0[r] - y0[center_h]);
            z[r][c] = exp(-(x_diff / (2 * sigma * sigma) + y_diff / (2 * sigma * sigma)));
        }
    }
    return z;
}

// Function to generate a vector of distances
vector<double> generateVecEDist(const vector<vector<double>>& map_e, const vector<vector<double>>& dist_e) {
    vector<double> vec_e_dist;
    int h = map_e.size();
    int w = map_e[0].size();
    
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            if (map_e[r][c] != 1.0) continue;
            vec_e_dist.push_back(dist_e[r][c]);
        }
    }
    return vec_e_dist;
}


// void readMap(const KivaGrid& map, vector<vector<double>>& map_e, vector<vector<double>>& map_w){
//     map_e.resize(map.rows, vector<double>(map.cols, 0));
//     map_w.resize(map.rows, vector<double>(map.cols, 0));
//     for (auto e_id: map.endpoints){
//         int r = e_id / map.cols;
//         int c = e_id % map.cols;
//         map_e[r][c] = 1;
//     }
//     for (auto w_id: map.workstations){
//         int r = w_id / map.cols;
//         int c = w_id % map.cols;
//         map_w[r][c] = 1;
//     }
// }

// Function to generate task and agent distribution
// void generateTaskAndAgentGaussianDist(const KivaGrid& map, mt19937& MT, vector<double>& w_dist, vector<double>& e_dist) {
//     vector<vector<double>> map_e, map_w;

//     readMap(map, map_e, map_w);
    
//     int num_w = 0;
//     for (const auto& row : map_w) {
//         num_w += std::count(row.begin(), row.end(), 1.0);
//     }

//     w_dist.resize(num_w, 1.0);
    
//     int h = map.rows;
//     int w = map.cols;

//     uniform_int_distribution<> dis_h(0, h - 1);
//     uniform_int_distribution<> dis_w(0, w - 1);
    
//     int center_h = dis_h(MT);
//     int center_w = dis_w(MT);
//     std::cout << "gaussian center = "<< center_h <<", " << center_w<<std::endl;
    
//     vector<vector<double>> dist_full = getGaussian(h, w, center_h, center_w);
//     vector<vector<double>> dist_e(h, vector<double>(w, 0));
//     double max_val = 0;

//     for (int r = 0; r < h; ++r) {
//         for (int c = 0; c < w; ++c) {
//             dist_e[r][c] = dist_full[r][c] * map_e[r][c];
//             if (dist_e[r][c] > max_val) max_val = dist_e[r][c];
//         }
//     }

//     for (int r = 0; r < h; ++r) {
//         for (int c = 0; c < w; ++c) {
//             dist_e[r][c] /= max_val; // normalize
//         }
//     }

//     e_dist = generateVecEDist(map_e, dist_e);

//     // For demonstration purposes
//     // cout << "Task and Agent Distribution Generated Successfully!" << endl;
// }

// void generateTaskAndAgentGaussianEmptyDist(const KivaGrid& map, mt19937& MT, vector<double>& empty_weights) {
    
//     vector<vector<double>> map_e(map.rows, vector<double>(map.cols, 0));
//     for (auto loc: map.empty_locations){
//         int r = loc / map.cols;
//         int c = loc % map.cols;
//         map_e[r][c] = 1;
//     }

//     int h = map.rows;
//     int w = map.cols;

//     random_device rd;
//     uniform_int_distribution<> dis_h(0, h - 1);
//     uniform_int_distribution<> dis_w(0, w - 1);
    
//     int center_h = dis_h(MT);
//     int center_w = dis_w(MT);

//     std::cout << "gaussian center = "<< center_h <<", " << center_w<<std::endl;
    
//     vector<vector<double>> dist_full = getGaussian(h, w, center_h, center_w);
//     vector<vector<double>> dist_e(h, vector<double>(w, 0));
//     double max_val = 0;

//     for (int r = 0; r < h; ++r) {
//         for (int c = 0; c < w; ++c) {
//             dist_e[r][c] = dist_full[r][c] * map_e[r][c];
//             if (dist_e[r][c] > max_val) max_val = dist_e[r][c];
//         }
//     }

//     for (int r = 0; r < h; ++r) {
//         for (int c = 0; c < w; ++c) {
//             dist_e[r][c] /= max_val; // normalize
//         }
//     }

//     empty_weights = generateVecEDist(map_e, dist_e);
// }

// void generateMultiModeGaussianEmptyDist(const KivaGrid& map, mt19937& MT,  vector<double>& empty_weights, int K){
//     std::vector<std::vector<double>> all_weights(K);
//     for (int i=0; i<K; ++i){
//         generateTaskAndAgentGaussianEmptyDist(map, MT, all_weights[i]);
//     }
//     empty_weights.clear();
//     for (int i=0; i<all_weights[0].size(); ++i){
//         double w = 0;
//         for (int k=0; k<K; ++k){
//             w += all_weights[k][i];
//         }
//         empty_weights.push_back(w);
//     }
// }


// void generateTaskAndAgentLRDist(const KivaGrid& map, mt19937& MT, vector<double>& w_dist, vector<double>& e_dist) {
//     double left_right_ratio_bound = 0.1;
//     std::uniform_real_distribution<> dis(0.0, 1.0);
//     double p = dis(MT);
//     double r0 = dis(MT) * (1-left_right_ratio_bound) + left_right_ratio_bound;
//     double ratio = (p<0.5)? r0: 1/r0;

//     w_dist.clear();
//     for (auto w_id: map.workstations){
//         int r = w_id / map.cols;
//         int c = w_id % map.cols;
//         if (c==0){
//             w_dist.push_back(ratio);
//         } else {
//             w_dist.push_back(1.0);
//         }
//     }

//     e_dist.resize(map.endpoints.size(), 1.0);
// }
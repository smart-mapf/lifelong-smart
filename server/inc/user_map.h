#pragma once

#include "common.h"
#include <boost/tokenizer.hpp>
#include <random>


class userMap {
public:
    userMap(std::string& map_fname);
    bool inline isValid(int x, int y) {
        return my_map[x][y];
    }
    bool readMap(std::string& map_fname);

    std::pair<int, int> findRandomPos() {
        std::mt19937 rng(static_cast<unsigned>(std::time(nullptr)));  // random number generator
        std::uniform_int_distribution<int> dist_row(0, num_of_rows - 1);
        std::uniform_int_distribution<int> dist_col(0, num_of_cols - 1);

        int x, y;
        do {
            x = dist_row(rng);
            y = dist_col(rng);
        } while (!isValid(x, y));  // keep trying until a valid position is found

        return std::make_pair(x, y);
    }

public:
    std::vector<std::shared_ptr<Station>> all_stations;
    std::vector<std::shared_ptr<Pod>> all_pods;
    std::vector< std::vector< std::shared_ptr<Pod> > > pods_by_genre;
    int num_of_rows;
    int num_of_cols;

private:
    std::vector<std::vector<bool>> my_map;
};
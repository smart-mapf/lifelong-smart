#pragma once

#include "common.h"
#include <boost/tokenizer.hpp>
#include <random>
#include <unordered_set>

struct FreeCell {
    std::pair<int, int> position;
    bool occupied = false;
    FreeCell(int x, int y) : position(x, y) {}
};

class userMap {
public:
    userMap(std::string& map_fname);
    bool inline isValid(int x, int y) {
        if (x >= num_of_rows or x <0) {
            return false;
        } else if (y >= num_of_cols or y < 0) {

        }
        return my_map[x][y];
    }
    bool readMap(std::string& map_fname);

public:
    std::vector<std::shared_ptr<Pod>> all_pods;
    std::vector< std::vector< std::shared_ptr<Pod> > > pods_by_genre;
    int num_of_rows;
    int num_of_cols;

private:
    std::vector<std::vector<bool>> my_map;
    std::vector<FreeCell> free_cells;
};
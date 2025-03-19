#pragma once

#include "common.h"
#include <boost/tokenizer.hpp>

class userMap {
public:
    userMap(std::string& map_fname);
    bool inline isValid(int x, int y) {
        return my_map[x][y];
    }
    bool readMap(std::string& map_fname);

public:
    std::vector<std::shared_ptr<Station>> all_stations;
    std::vector<std::shared_ptr<Pod>> all_pods;
    std::vector< std::vector< std::shared_ptr<Pod> > > pods_by_genre;
    int num_of_rows;
    int num_of_cols;

private:
    std::vector<std::vector<bool>> my_map;
};
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

    std::pair<int, int> findRandomPos(unordered_set<std::pair<int, int>, pair_hash>& occupied_locs_set) {
        std::mt19937 rng(static_cast<unsigned>(std::time(nullptr)));  // random number generator
        std::uniform_int_distribution<int> dist_idx(0, free_cells.size() - 1);

        int idx;
        std::pair<int, int> pos;
        do {
            idx = dist_idx(rng);
            pos = free_cells[idx].position;
        } while (occupied_locs_set.contains(pos));  // keep trying until a valid position is found

        return pos;
    }

    std::pair<int, int> findNeighborPos(unordered_set<std::pair<int, int>, pair_hash>& occupied_locs_set, std::pair<int, int> pos) {
        // Directions: up, down, left, right
        const int dx[] = {-1, 1, 0, 0};
        const int dy[] = {0, 0, -1, 1};

        // Set to track visited positions
        unordered_set<pair<int, int>, pair_hash> visited;
        std::deque<pair<int, int>> q;

        q.push_back(pos);
        visited.insert(pos);

        while (!q.empty()) {
            auto current = q.front();
            q.pop_front();

            for (int i = 0; i < 4; ++i) {
                pair<int, int> neighbor = {current.first + dx[i], current.second + dy[i]};

                if (visited.find(neighbor) != visited.end()) continue;
                visited.insert(neighbor);

                if (occupied_locs_set.find(neighbor) == occupied_locs_set.end() and
                    isValid(neighbor.first, neighbor.second) and
                    not isStation(neighbor)) {
                    return neighbor;
                }

                q.push_back(neighbor);
            }
        }

        // Should never reach here unless all positions are somehow occupied
        return pos;
    }

    /**
     *
     * @param pos x: index of rows, y: index of columns
     * @return
     */
    bool isStation(std::pair<int, int> pos) {
        for (auto& station: all_stations) {
            // std::cout << "pos x is: " << pos.first << " y is: " << pos.second << ", while the station x is: "
            // << station->x << " the station y is: " << station->y << std::endl;
            if (station->x == pos.first and station->y == pos.second) {
                return true;
            }
        }
        return false;
    }

public:
    std::vector<std::shared_ptr<Station>> all_stations;
    std::vector<std::shared_ptr<Pod>> all_pods;
    std::vector< std::vector< std::shared_ptr<Pod> > > pods_by_genre;
    int num_of_rows;
    int num_of_cols;

private:
    std::vector<std::vector<bool>> my_map;
    std::vector<FreeCell> free_cells;
};
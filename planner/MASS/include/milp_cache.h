/**
 * @brief Implement of the Success/Failure cache
 *
*/
#pragma once

#include <iostream>
#include <vector>
#include <utility>

#include "instance.h"

#define CACHE_MAX_SIZE 9999

typedef std::vector<std::pair<double, double>> EntryKey;

struct CacheEntry{
    EntryKey keys;
    std::shared_ptr<MotionNode> bezier_val = nullptr;
    inline bool IsWithinKey(const EntryKey& query_key);
    inline bool IsEqual(const EntryKey& query_key);
};



/**
 * @brief Change the time interval sequence to the key in the cache
 *
 * @param path Time interval sequence need to be transformed
 * @param key[out] Correspond key to the time interval sequence
 * @return None
 */
inline static bool PathToKey(IntervalSeq &path, EntryKey &key)
{
    for (std::shared_ptr<IntervalEntry>& elem: path) {
        key.emplace_back(elem->t_min, elem->t_max);
    }
    return true;
}

class SuccessCache{
public:
    SuccessCache() { cache_size = CACHE_MAX_SIZE; }
    bool InsertEntry(IntervalSeq& new_path, std::shared_ptr<MotionNode>& bezier_solution);
    bool FindEntry(IntervalSeq& query_path, std::shared_ptr<MotionNode>& store_solution);
    void reset() {cache_table.clear();}

private:
    std::unordered_map<size_t, std::deque<CacheEntry>> cache_table;
    size_t cache_size;
};

class FailureCache{
public:
    bool InsertEntry(IntervalSeq& new_path);
    bool FindEntry(IntervalSeq& query_path);
    void reset() {cache_table.clear();}

private:
    std::unordered_map<size_t, std::deque<CacheEntry>> cache_table;
    size_t cache_size;
};
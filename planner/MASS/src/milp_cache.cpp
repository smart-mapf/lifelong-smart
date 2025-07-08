#include "milp_cache.h"

/**
 * @brief Determine if the current time interval sequence contains the query time interval sequence
 *
 * @param query_key The query time interval sequence
 * @return If the query key lies within current interval sequence
 */
bool CacheEntry::IsWithinKey(const EntryKey& query_key)
{
    bool flag = true;
    for(size_t i = 0; i < query_key.size(); i++) {
        if (query_key[i].first < keys[i].first ||
            query_key[i].second > keys[i].second) {
                flag = false;
                break;
            }
    }
    return flag;
}

/**
 * @brief Determine if the current time interval sequence equals the query time interval sequence
 *
 * @param query_key The query time interval sequence
 * @return If the query key equals exactly to the current interval sequence
 */
bool CacheEntry::IsEqual(const EntryKey& query_key)
{
    bool flag = true;
    for(size_t i = 0; i < query_key.size(); i++) {
        if (query_key[i].first != keys[i].first ||
            query_key[i].second != keys[i].second) {
                flag = false;
                break;
            }
    }
    return flag;
}

/**
 * @brief Insert the Interval sequence and the BCP solution to the cache
 *
 * @param new_path The interval sequence that we want to insert to the cache
 * @param bezier_solution The corresponding solution from BCP
 * @return If the insert operation succeed
 */
bool SuccessCache::InsertEntry(IntervalSeq& new_path, std::shared_ptr<MotionNode>& bezier_solution) {
    size_t traj_size = new_path.size();
    CacheEntry new_entry;
    PathToKey(new_path, new_entry.keys);
    new_entry.bezier_val = bezier_solution;
    if (cache_table.find(new_path.size()) == cache_table.end()) {
        cache_table[traj_size] = std::deque<CacheEntry> {};
    }
    cache_table[traj_size].push_back(new_entry);

    if (cache_table[traj_size].size() >= cache_size) {
        cache_table[traj_size].pop_front();
    }
    return true;
}

/**
 * @brief Find the time interval sequence in the success cache and return the stored BCP
 * solution if one is found.
 *
 * @param query_path The time interval we want to query
 * @param store_solution[out] The Bezier solution found
 * @return True is query path is found, false otherwise
 */
bool SuccessCache::FindEntry(IntervalSeq &query_path, shared_ptr<MotionNode> &store_solution) {
    size_t traj_length = query_path.size();
    if (cache_table.find(traj_length) != cache_table.end()) {
        EntryKey query_key;
        PathToKey(query_path, query_key);
        for(CacheEntry tmp_entry: cache_table[traj_length]) {
            if (tmp_entry.IsEqual(query_key)) {
                store_solution = tmp_entry.bezier_val;
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief Insert the Interval sequence to the failure cache
 *
 * @param new_path The interval sequence that we want to insert to the cache
 * @return If the insert operation succeed
 */
bool FailureCache::InsertEntry(IntervalSeq &new_path) {
    size_t traj_size = new_path.size();
    CacheEntry new_entry;
    PathToKey(new_path, new_entry.keys);
    if (cache_table.find(new_path.size()) == cache_table.end()) {
        cache_table[traj_size] = std::deque<CacheEntry> {};
    }
    cache_table[traj_size].push_back(new_entry);

    if (cache_table[traj_size].size() >= cache_size) {
        cache_table[traj_size].pop_front();
    }
    return true;
}

/**
 * @brief Find the time interval sequence in the failure cache
 *
 * @param query_path The time interval we want to query
 * @return True is query path is found, false otherwise
 */
bool FailureCache::FindEntry(IntervalSeq &query_path) {
    size_t traj_length = query_path.size();
    if (cache_table.find(traj_length) != cache_table.end()) {
        EntryKey query_key;
        PathToKey(query_path, query_key);
        for(CacheEntry tmp_entry: cache_table[traj_length]) {
            if (tmp_entry.IsWithinKey(query_key)) {
                return true;
            }
        }
    }
    return false;
}

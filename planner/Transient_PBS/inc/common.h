#pragma once
#include <boost/filesystem.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/pairing_heap.hpp>
#include <boost/tokenizer.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <cfloat>
#include <ctime>
#include <fstream>
#include <iomanip>   // std::setprecision
#include <iostream>  // std::cout, std::fixed
#include <json.hpp>
#include <list>
#include <map>
#include <random>
#include <set>
#include <stack>
#include <tuple>
#include <vector>

#include "spdlog/spdlog.h"

using boost::char_separator;
using boost::tokenizer;
using boost::unordered_map;
using boost::unordered_set;
using boost::heap::compare;
using boost::heap::fibonacci_heap;
using boost::heap::pairing_heap;
using std::cerr;
using std::clock;
using std::cout;
using std::endl;
using std::get;
using std::getline;
using std::ifstream;
using std::list;
using std::make_pair;
using std::make_shared;
using std::make_tuple;
using std::map;
using std::max;
using std::min;
using std::mt19937;
using std::ofstream;
using std::pair;
using std::set;
using std::shared_ptr;
using std::stack;
using std::string;
using std::tie;
using std::to_string;
using std::tuple;
using std::vector;

using json = nlohmann::json;

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES INT_MAX / 2
#define WEIGHT_MAX INT_MAX / 2

struct PathEntry {
    int location = -1;

    // Record the sum of costs at the end of the path.
    double sum_of_costs = -1;
    explicit PathEntry(int loc = -1) : location(loc) {
    }
};

typedef vector<PathEntry> Path;
std::ostream& operator<<(std::ostream& os, const Path& path);

bool isSamePath(const Path& p1, const Path& p2);

bool congested(const vector<vector<tuple<int, int, double, int>>>& new_plan);

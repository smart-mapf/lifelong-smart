#pragma once
#include <boost/algorithm/string.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/tokenizer.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <cfloat>
#include <ctime>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <random>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "spdlog/spdlog.h"
#include "json.hpp"

using boost::unordered_map;
using boost::unordered_set;
using boost::heap::compare;
using boost::heap::fibonacci_heap;
using json = nlohmann::json;

using std::cout;
using std::deque;
using std::discrete_distribution;
using std::endl;
using std::list;
using std::make_pair;
using std::make_shared;
using std::make_tuple;
using std::map;
using std::max;
using std::min;
using std::mt19937;
using std::ofstream;
using std::ostream;
using std::pair;
using std::queue;
using std::set;
using std::string;
using std::to_string;
using std::tuple;
using std::vector;

// #include <boost/graph/adjacency_list.hpp>
// typedef boost::adjacency_list_traits<int, int, boost::undirectedS >
// confilctGraph_t; typedef confilctGraph_t::vertex_descriptor vertex_t; typedef
// confilctGraph_t::edge_descriptor edge_t;

enum heuristics_type { ZERO, CG, DG, WDG, STRATEGY_COUNT };

typedef tuple<int, int, int, int, bool> Constraint;
typedef tuple<int, int, int, int, int> Conflict;
// typedef vector<unordered_set<std::tuple<int, int, int> > > ConstraintTable;
typedef tuple<int, int, bool>
    Interval;  // [t_min, t_max), have conflicts or not
#define INTERVAL_MAX 10000

ostream& operator<<(ostream& os, const Constraint& constraint);

ostream& operator<<(ostream& os, const Conflict& conflict);

ostream& operator<<(ostream& os, const Interval& interval);

// According to https://stackoverflow.com/a/40854664, we need to add a
// operator<<() for vector<int> so that boost can automatically cast the
// default vector parameter value. We need this specifically for the
// `station_wait_times` command line argument.
namespace std {
ostream& operator<<(ostream& os, const vector<int>& vec);
}

vector<list<int>> read_int_vec(string fname, int team_size);
vector<list<tuple<int, int, int, int>>> read_tuple_vec(string fname,
                                                       int team_size);

int select_min_key_random_tie(const boost::unordered_map<int, int>& m,
                              int seed);
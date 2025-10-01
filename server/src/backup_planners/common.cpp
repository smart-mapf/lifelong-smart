#include "backup_planners/common.h"

bool validMove(int curr, int next, int map_size, int num_col) {
    if (next < 0 || next >= map_size)
        return false;
    int curr_x = curr / num_col;
    int curr_y = curr % num_col;
    int next_x = next / num_col;
    int next_y = next % num_col;
    return abs(next_x - curr_x) + abs(next_y - curr_y) < 2;
}

std::ostream& operator<<(std::ostream& os, const Constraint& constraint) {
    os << "<" << std::get<0>(constraint) << "," << std::get<1>(constraint)
       << "," << std::get<2>(constraint) << "," << std::get<3>(constraint)
       << "," << std::get<4>(constraint) << ">";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Conflict& conflict) {
    os << "<" << std::get<0>(conflict) << "," << std::get<1>(conflict) << ","
       << std::get<2>(conflict) << "," << std::get<3>(conflict) << ","
       << std::get<4>(conflict) << ">";
    return os;
}

ostream& operator<<(ostream& os, const Interval& interval) {
    os << "[" << std::get<0>(interval) << "," << std::get<1>(interval) << ")("
       << std::get<2>(interval) << ")";
    return os;
}

namespace std {
ostream& operator<<(ostream& os, const vector<int>& vec) {
    for (auto item : vec) {
        os << item << " ";
    }
    return os;
}
}  // namespace std

vector<list<int>> read_int_vec(string fname, int team_size) {
    std::vector<list<int>> res;
    string line;
    std::ifstream myfile(fname.c_str());
    if (!myfile.is_open())
        return {};

    getline(myfile, line);
    while (!myfile.eof() && line[0] == '#') {
        getline(myfile, line);
    }

    boost::char_separator<char> sep(",");
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();

    int max_team_size = atoi((*beg).c_str());
    if (max_team_size < team_size) {
        throw std::runtime_error(
            "Input file wrong, no enough agents in agent file");
    }
    // My benchmark
    for (int i = 0; i < team_size; i++) {
        getline(myfile, line);
        while (!myfile.eof() && line[0] == '#') {
            getline(myfile, line);
        }
        boost::tokenizer<boost::char_separator<char>> tok(line, sep);
        boost::tokenizer<boost::char_separator<char>>::iterator beg =
            tok.begin();
        list<int> locs;
        for (; beg != tok.end(); ++beg) {
            locs.push_back(atoi((*beg).c_str()));
        }
        res.push_back(locs);
    }
    myfile.close();
    return res;
}

vector<list<tuple<int, int, int, int>>> read_tuple_vec(string fname,
                                                       int team_size) {
    std::vector<list<tuple<int, int, int, int>>> res;
    string line;
    std::ifstream myfile(fname.c_str());
    if (!myfile.is_open())
        return {};

    getline(myfile, line);
    while (!myfile.eof() && line[0] == '#') {
        getline(myfile, line);
    }

    boost::char_separator<char> sep(",");
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);
    boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin();

    int max_team_size = atoi((*beg).c_str());
    if (max_team_size < team_size) {
        throw std::runtime_error(
            "Input file wrong, no enough agents in agent file");
    }
    for (int i = 0; i < team_size; i++) {
        getline(myfile, line);
        while (!myfile.eof() && line[0] == '#') {
            getline(myfile, line);
        }
        // Split by `;`, each element is a tuple of (loc, ori, wait timesteps)
        vector<string> results;
        boost::split(results, line, boost::is_any_of(";"));
        list<tuple<int, int, int, int>> tasks;
        for (auto& raw_task : results) {
            if (raw_task.empty()) {
                continue;
            }
            // Split by `,`, each element is (loc, ori, wait timesteps)
            vector<string> task;
            boost::split(task, raw_task, boost::is_any_of(","));
            // cout << raw_task << endl;
            if (task.size() != 4) {
                throw std::runtime_error(
                    "Input file wrong, each task should be a tuple of "
                    "(loc, ori, wait_t, move)");
            }
            int loc = atoi(task[0].c_str());
            int ori = atoi(task[1].c_str());
            int wait_time = atoi(task[2].c_str());
            int move = atoi(task[3].c_str());
            tasks.push_back(make_tuple(loc, ori, wait_time, move));
        }
        res.push_back(tasks);
    }
    return res;
}

int select_min_key_random_tie(const boost::unordered_map<int, int>& m,
                              int seed) {
    // Step 1: find minimum value using std::min_element
    auto min_it = std::min_element(
        m.begin(), m.end(),
        [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
            return a.second < b.second;
        });
    int min_val = min_it->second;

    // Step 2: collect all keys with min_val
    std::vector<int> candidates;
    for (const auto& kv : m) {
        if (kv.second == min_val)
            candidates.push_back(kv.first);
    }

    // Step 3: break tie randomly
    std::mt19937 gen(seed);
    std::uniform_int_distribution<> dist(0, candidates.size() - 1);
    return candidates[dist(gen)];
}
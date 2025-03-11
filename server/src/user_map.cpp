#include "user_map.h"

userMap::userMap(std::string& map_fname) {
    bool success = readMap(map_fname);
    if (!success) {
        std::cerr << "Error reading map file " << map_fname << std::endl;
        exit(-1);
    }
}


bool userMap::readMap(std::string& map_fname) {
    using namespace boost;
    using namespace std;
    ifstream myfile(map_fname.c_str());
    if (!myfile.is_open())
        return false;
    string line;
    tokenizer< char_separator<char> >::iterator beg;
    getline(myfile, line);
    if (line[0] == 't') // Nathan's benchmark
    {
        char_separator<char> sep(" ");
        getline(myfile, line);
        tokenizer< char_separator<char> > tok(line, sep);
        beg = tok.begin();
        beg++;
        num_of_rows = atoi((*beg).c_str()); // read number of rows
        getline(myfile, line);
        tokenizer< char_separator<char> > tok2(line, sep);
        beg = tok2.begin();
        beg++;
        num_of_cols = atoi((*beg).c_str()); // read number of cols
        getline(myfile, line); // skip "map"
    } else {
        return false;
    }
    // map_size = num_of_cols * num_of_rows;
    my_map.resize(num_of_rows, std::vector<bool> (num_of_cols, false));
    // read map (and start/goal locations)
    int station_id = 0;
    int pod_id = 0;
    for (int i = 0; i < num_of_rows; i++) {
        getline(myfile, line);
        for (int j = 0; j < num_of_cols; j++) {
            // true for a free cell, false otherwise
            my_map[i][j] = (line[j] == '.');
            if (line[j] == 'T') {
                // If station
                my_map[i][j] = true;
                all_stations.emplace_back(std::make_shared<Station>(i, j, 0, station_id));
                station_id++;
            } else if (line[j] == 'P') {
                // If Pods
                all_pods.emplace_back(std::make_shared<Pod>(i, j, 0, pod_id));
                pod_id++;
            }
        }
    }
    myfile.close();
    return true;
}
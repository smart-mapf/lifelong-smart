#include "ManufactureGraph.h"
#include <fstream>
#include <boost/tokenizer.hpp>
#include "StateTimeAStar.h"
#include <sstream>
#include <random>
#include <chrono>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;


bool ManufactureGrid::load_map_from_jsonstr(std::string G_json_str)
{
	json G_json = json::parse(G_json_str);
	if (G_json["weight"])
		return load_weighted_map_from_json(G_json);
	else
		return load_unweighted_map_from_json(G_json);
}

// Infer the endpoint types
void ManufactureGrid::infer_endpoint_type()
{
    for (int i = 0; i < this->rows; i++)
	{
		for (int j = 0; j < this->cols; j++)
        {
			int id = this->cols * i + j;
			if (this->types[id] == "Endpoint")
			{
				for (int dir = 0; dir < 4; dir++)
				{
					int adj_id = id + this->move[dir];
					// If the move is valid
					if (0 <= adj_id &&
						adj_id < this->cols * this->rows &&
						get_Manhattan_distance(id, adj_id) <= 1)
					{
						for (int p = 0; p < this->n_station_types; p++)
						{
							string curr_type = "Manufacture_" + to_string(p);
							if (this->types[adj_id] == curr_type)
							{
								this->endpoints[p].push_back(id);
							}
						}
					}
				}
			}
        }
    }
}

// Process a line of map from either json or .map file
void ManufactureGrid::process_map_line(string line, int i)
{
	for (int j = 0; j < this->cols; j++)
	{
		int id = this->cols * i + j;
		this->weights[id].resize(5, WEIGHT_MAX);

		if (line[j] == 'e')
		{
			// The layout generator does not distinguish the endpoints.
			// We need to "infer" the type of endpoint after reading in the
			// entire map.
			// Note that one endpoint could belong to multiple different
			// types if it is adjacent to multiple manufacture workstations.
			this->types[id] = "Endpoint";
			this->weights[id][4] = 1;
			this->agent_home_locations.push_back(id);
		}
		else if(line[j] == '.')
		{
			this->types[id] = "Travel";
			this->weights[id][4] = 1;
			this->agent_home_locations.push_back(id);
		}
		else
		{
			// Read in the manufacture stations
			// Since in the map file we use the char of numbers to indicate the
			// type of manufacture workstations, it currently only supports up
			// to 10 (indexed as 0~9) types of manufacture stations.
			assert(this->n_station_types > 0 && this->n_station_types <= 10);
			for (int p = 0; p < this->n_station_types; p++)
			{
				string curr_type = std::to_string(p);
				if (line[j] == curr_type.c_str()[0])
				{
					this->types[id] = "Manufacture_" + curr_type;
					this->weights[id][4] = 1;
					this->manufacture_stations[p].push_back(id);
					break;
				}
			}
		}
	}
}

// Set the weight of the unweighted graph.
// The graph is "unweighted" s.t. all weights are 1 and weights towards the
// obstacles are WEIGHT_MAX.
void ManufactureGrid::set_unweighted_graph_weight()
{
	for (int i = 0; i < this->cols * this->rows; i++)
	{
		if (boost::starts_with(this->types[i], "Manufacture"))
		{
			continue;
		}
		for (int dir = 0; dir < 4; dir++)
		{
			int adj_id = i + this->move[dir];
			if (0 <= adj_id && adj_id < this->cols * this->rows &&
				get_Manhattan_distance(i, adj_id) <= 1 &&
				!boost::starts_with(this->types[adj_id], "Manufacture"))
				this->weights[i][dir] = 1;
			else
				this->weights[i][dir] = WEIGHT_MAX;
		}
	}
}


bool ManufactureGrid::load_unweighted_map_from_json(json G_json)
{
	std::cout << "*** Loading map ***" << std::endl;
    clock_t t = std::clock();

	// Read in n_row, n_col
	this->rows = G_json["n_row"];
	this->cols = G_json["n_col"];
	this->move[0] = 1;
	this->move[1] = -cols;
	this->move[2] = -1;
	this->move[3] = cols;
	this->map_name = G_json["name"];

	this->types.resize(this->rows * this->cols);
	this->weights.resize(this->rows * this->cols);
	this->manufacture_stations.resize(this->n_station_types, vector<int>());
	this->endpoints.resize(this->n_station_types, vector<int>());

	std::string line;

	for (int i = 0; i < this->rows; i++)
	{
		// getline(myfile, line);
		line = G_json["layout"][i];
		process_map_line(line, i);
	}

    // Infer the endpoint types
	infer_endpoint_type();

	shuffle(
		this->agent_home_locations.begin(),
		this->agent_home_locations.end(),
		std::default_random_engine());
	set_unweighted_graph_weight();

    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
	print_map_info();
    std::cout << "Done! (" << runtime << " s)" << std::endl;
	return true;
}

void ManufactureGrid::print_map_info()
{
	std::cout << "Map size: " << this->rows << "x" << this->cols << " with ";

	for (int p = 0; p < this->n_station_types; p++)
	{
		cout << this->manufacture_stations[p].size()
			 << " manufacture station " << p << ", ";
	}
	cout << endl;
}

bool ManufactureGrid::load_weighted_map_from_json(json G_json)
{
	// TODO
	return false;
}



bool ManufactureGrid::load_map(std::string fname)
{
    std::size_t pos = fname.rfind('.');      // position of the file extension
    auto ext_name = fname.substr(pos, fname.size());     // get the name without extension
    if (ext_name == ".grid")
        return load_weighted_map(fname);
    else if (ext_name == ".map")
        return load_unweighted_map(fname);
    else
    {
        std::cout << "Map file name should end with either .grid or .map. " << std::endl;
        return false;
    }
}

bool ManufactureGrid::load_weighted_map(std::string fname)
{
	// std::string line;
	// std::ifstream myfile((fname).c_str());
	// if (!myfile.is_open())
	// {
	// 	std::cout << "Map file " << fname << " does not exist. " << std::endl;
	// 	return false;
	// }

	// std::cout << "*** Loading map ***" << std::endl;
	// clock_t t = std::clock();
	// std::size_t pos = fname.rfind('.');      // position of the file extension
	// map_name = fname.substr(0, pos);     // get the name without extension
	// getline(myfile, line); // skip the words "grid size"
	// getline(myfile, line);
	// boost::char_separator<char> sep(",");
	// boost::tokenizer< boost::char_separator<char> > tok(line, sep);
	// boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
	// this->rows = atoi((*beg).c_str()); // read number of cols
	// beg++;
	// this->cols = atoi((*beg).c_str()); // read number of rows
	// move[0] = 1;
	// move[1] = -cols;
	// move[2] = -1;
	// move[3] = cols;

	// getline(myfile, line); // skip the headers

	// //read tyeps and edge weights
	// this->types.resize(rows * cols);
	// this->weights.resize(rows * cols);
	// for (int i = 0; i < rows * cols; i++)
	// {
	// 	getline(myfile, line);
	// 	boost::tokenizer< boost::char_separator<char> > tok(line, sep);
	// 	beg = tok.begin();
	// 	beg++; // skip id
	// 	this->types[i] = std::string(beg->c_str()); // read type
	// 	beg++;
	// 	if (types[i] == "Home")
	// 		this->agent_home_locations.push_back(i);
	// 	else if (types[i] == "Endpoint")
	// 		this->manufacture_1.push_back(i);
	// 	beg++; // skip x
	// 	beg++; // skip y
	// 	weights[i].resize(5);
	// 	for (int j = 0; j < 5; j++) // read edge weights
	// 	{
	// 		if (std::string(beg->c_str()) == "inf")
	// 			weights[i][j] = WEIGHT_MAX;
	// 		else
	// 			weights[i][j] = std::stod(beg->c_str());
	// 		beg++;
	// 	}
	// }

	// myfile.close();
	// double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
	// std::cout << "Map size: " << rows << "x" << cols << " with ";
    // std::cout << this->manufacture_1.size() << " manufacture A and "
    // << this->manufacture_2.size() << " manufacture B." << endl;
	// std::cout << "Done! (" << runtime << " s)" << std::endl;
	// return true;
    return false;
}


// load map
bool ManufactureGrid::load_unweighted_map(std::string fname)
{
    std::string line;
    std::ifstream myfile ((fname).c_str());
	if (!myfile.is_open())
    {
	    std::cout << "Map file " << fname << " does not exist. " << std::endl;
        return false;
    }

    std::cout << "*** Loading map ***" << std::endl;
    clock_t t = std::clock();
	std::size_t pos = fname.rfind('.');      // position of the file extension
    map_name = fname.substr(0, pos);     // get the name without extension
    getline (myfile, line);


	boost::char_separator<char> sep(",");
	boost::tokenizer< boost::char_separator<char> > tok(line, sep);
	boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
	this->rows = atoi((*beg).c_str()); // read number of rows
	beg++;
	this->cols = atoi((*beg).c_str()); // read number of cols
	this->move[0] = 1;
	this->move[1] = -cols;
	this->move[2] = -1;
	this->move[3] = cols;

	this->types.resize(rows * cols);
	this->weights.resize(rows*cols);
	this->manufacture_stations.resize(this->n_station_types, vector<int>());
	this->endpoints.resize(this->n_station_types, vector<int>());

	for (int i = 0; i < rows; i++)
	{
		getline(myfile, line);
		process_map_line(line, i);
	}

    // Infer the endpoint types
	infer_endpoint_type();

	shuffle(
		this->agent_home_locations.begin(),
		this->agent_home_locations.end(),
		std::default_random_engine());
	set_unweighted_graph_weight();

	myfile.close();
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
	print_map_info();
    std::cout << "Done! (" << runtime << " s)" << std::endl;
    return true;
}

void ManufactureGrid::preprocessing(bool consider_rotation, std::string log_dir)
{
	std::cout << "*** PreProcessing map ***" << std::endl;
	clock_t t = std::clock();
	this->consider_rotation = consider_rotation;
	fs::path table_save_path(log_dir);
	if (consider_rotation)
		table_save_path /= map_name + "_rotation_heuristics_table.txt";
	else
		table_save_path /= map_name + "_heuristics_table.txt";
	std::ifstream myfile(table_save_path.c_str());
	bool succ = false;
	if (myfile.is_open())
	{
		succ = load_heuristics_table(myfile);
		myfile.close();
	}
	if (!succ)
	{
        // Compute heuristic from endpoints
		for (int p = 0; p < this->n_station_types; p++)
		{
			for (auto endpoint : this->endpoints[p])
			{
				this->heuristics[endpoint] = compute_heuristics(endpoint);
                this->pebble_motion_heuristics[endpoint] =
                    compute_pebble_motion_heuristics(endpoint);
			}
		}
		cout << table_save_path << endl;
		save_heuristics_table(table_save_path.string());
	}

	double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
	std::cout << "Done! (" << runtime << " s)" << std::endl;
}

double ManufactureGrid::get_avg_task_len(
    unordered_map<int, vector<double>> heuristics) const
{
    double total_task_len = 0.0;
    int n_tasks = 0;

	for (int p = 0; p < this->n_station_types; p++)
	{
		int next_p = (p + 1) % this->n_station_types;
		for (auto endpoint1 : this->endpoints[p])
		{
			for (auto endpoint2 : this->endpoints[next_p])
			{
				if (endpoint1 != endpoint2)
				{
					total_task_len += heuristics[endpoint1][endpoint2];
				}
				// Note: even if endpoint1 and endpoint2 are the same, it is
				// considered a task in manufacture scenario.
				n_tasks += 1;
			}
		}
	}
    return total_task_len / n_tasks;
}


bool ManufactureGrid::load_map_from_jsonstr(std::string json_str, double left_w_weight, double right_w_weight)
{
    throw std::runtime_error("ManufactureGrid::load_map_from_jsonstr Not implemented");
}

void ManufactureGrid::reset_weights(bool consider_rotation, std::string log_dir, bool optimize_wait, std::vector<double>& weights)
{
    throw std::runtime_error("ManufactureGrid::reset_weights Not implemented");
}
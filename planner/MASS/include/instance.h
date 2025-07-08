#pragma once

#include "common.h"
#include <vector>
#include<boost/tokenizer.hpp>
#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>

using namespace std;

struct position_t
{
    float x;
    float y;
    position_t()=default;
    position_t(float a, float b) {
        x = a;
        y = b;
    };
};

struct Vertex
{
    std::string name;
    position_t pos;
    std::set<int> generalizedVertexConflicts;
    int id;
};

struct Edge
{
    std::string name;
    float length;
};

struct Node
{
    bool is_expanded = false;
    int current_point;
    Action prev_action;
    orient curr_o;

    int interval_index;
    double arrival_time_min; // arrival time of the head of the vehicle
    double arrival_time_max; // arrival time of the head of the vehicle
    
	double g = 0;      //
	double f = 0;      // f = h + g;
	double h = 0;      //

    std::shared_ptr<MotionNode> bezier_solution = nullptr;
    std::shared_ptr<Node> parent;
    IntervalQueue partial_intervals;
};

class NodeCompare
{
public:
    bool operator() (const std::shared_ptr<Node>& N1, const std::shared_ptr<Node>& N2) {
        if (N1->f > N2->f) {
            return true;
        } else {
            return false;
        }
    }
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Vertex, Edge> searchGraph_t;

struct Agent
{
    int id = -1;
    int start_location = -1;
    orient start_o = orient::East;
    int goal_location = -1;
    orient end_o = orient::East;

	double v_min = V_MIN;
    double v_max = V_MAX;
    double a_min = -0.5;
    double a_max = 0.5;
	double length = 0.4; // radius of the robot
    double rotation_cost = ROTATE_COST;
    double turn_back_cost = TURN_BACK_COST;

    /*For time window*/
    bool is_solved = false;
    double earliest_start_time = 0.0;
    double init_velocity = 0.0;
	Path previous_path;
    std::vector<int> trajectory;

	Agent(int start_loc = -1, int goal_loc = -1)
	{
		start_location = start_loc;
		goal_location = goal_loc;
	}

    /*statistic for result*/
    double total_cost = 0;
    double tmp_cost = 0;
};

struct Neighbors{
    pair<int, orient> left;
    pair<int, orient> right;
    pair<int, orient> back;
    std::vector<int> forward_locs;
};

// Currently only works for undirected unweighted 4-neighbor grids
class Instance 
{
public:
	int num_of_cols = 0;
	int num_of_rows = 0;
	int map_size = 0;
	std::vector<Agent> agents;
    int num_of_agents = 0;
    int use_sps_type = 0; // 0 for Binary, 1 for Bezier
    bool use_pe = false;


	Instance() = default;
	Instance(const string& map_fname, const string& agent_fname, 
		int num_of_agents = 0, const string& agent_indices = "",
		int num_of_rows = 0, int num_of_cols = 0, int num_of_obstacles = 0, int warehouse_width = 0,
                bool use_partial_expansion = false, int used_sps_solver = 0);

	void printAgents() const;

	void GetRawReservationTable();
	inline bool isObstacle(int loc) const { return my_map[loc]; }
	inline bool validMove(int curr, int next) const
	{
		if (next < 0 || next >= map_size)
			return false;
		if (my_map[next])
			return false;
		return getManhattanDistance(curr, next) < 2;
	}

	void finishAgent(int goal)
	{
		my_map[goal] = 1;
	}

	list<int> getNeighbors(int curr) const;
    void getNeighbors(int curr, orient curr_o, std::vector<std::tuple<int, orient, double>>& neighbors) const;
    void getNeighbors(int curr, orient direct, Neighbors& neighbor) const;
    void getInverseNeighbors(int curr, orient direct, Neighbors& neighbor) const;

    inline int linearizeCoordinate(int row, int col) const { return ( this->num_of_cols * row + col); }
	inline int getRowCoordinate(int id) const { return id / this->num_of_cols; }
	inline int getColCoordinate(int id) const { return id % this->num_of_cols; }
	inline pair<int, int> getCoordinate(int id) const { return make_pair(id / this->num_of_cols, id % this->num_of_cols); }
	inline int getCols() const { return num_of_cols; }
	inline int getRows() const { return num_of_rows; }

	inline int getManhattanDistance(int loc1, int loc2) const
	{
		int loc1_x = getRowCoordinate(loc1);
		int loc1_y = getColCoordinate(loc1);
		int loc2_x = getRowCoordinate(loc2);
		int loc2_y = getColCoordinate(loc2);
		return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
	}

	static inline int getManhattanDistance(const pair<int, int>& loc1, const pair<int, int>& loc2)
	{
		return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
	}

	int walkCounterClockwise(int from, int to) const
	{
		assert(validMove(from, to));
		int dir = turnLeft(to - from);
		while (!validMove(from, from + dir))
			dir = turnLeft(dir);
		return from + dir;
	}
	inline int turnLeft(int dir) const
	{
		if (dir ==  1) 
			return -num_of_cols;
		else if (dir == -num_of_cols)
			return - 1;
		else if (dir == -1)
			return num_of_cols;
		else if (dir == num_of_cols)
			return 1;
		else
			return 0;
	 }
	inline int turnRight(int dir) const
	{
		if (dir == 1)
			return num_of_cols;
		else if (dir == num_of_cols)
			return -1;
		else if (dir == -1)
			return -num_of_cols;
		else if (dir == -num_of_cols)
			return 1;
		else
			return 0;
	}
	int getDegree(int loc) const // return the number of neighbors
	{
		assert(loc >= 0 && loc < map_size && !my_map[loc]);
		int degree = 0;
		if (0 <= loc - num_of_cols && !my_map[loc - num_of_cols])
			degree++;
		if (loc + num_of_cols < map_size && !my_map[loc + num_of_cols])
			degree++;
		if (loc % num_of_cols > 0 && !my_map[loc - 1])
			degree++;
		if (loc % num_of_cols < num_of_cols - 1 && !my_map[loc + 1])
			degree++;
		return degree;
	}

	int getDefaultNumberOfAgents() const { return num_of_agents; }
	void GetAgents(std::vector<Agent>& agents_list);
	int GetNumOfVertices() {return map_size;};
	void GetReservationTable(ReservationTable& rt) {rt = raw_rv_tbl; };


private:
	// int moves_offset[MOVE_COUNT];
	vector<bool> my_map;
	string map_fname;
	string agent_fname;
	string agent_indices;
	ReservationTable raw_rv_tbl;

//   vector<int> start_locations;
//   vector<int> goal_locations;
	
private:
	bool loadMap();
	void printMap() const;
	void saveMap() const;

	bool loadAgents();
	void saveAgents() const;

	void generateConnectedRandomGrid(int rows, int cols, int obstacles); // initialize new [rows x cols] map with random obstacles
	void generateRandomAgents(int warehouse_width);
	bool addObstacle(int obstacle); // add this obsatcle only if the map is still connected
	bool isConnected(int start, int goal) const; // run BFS to find a path between start and goal, return true if a path exists.

	int randomWalk(int loc, int steps) const;


	// Class  SingleAgentSolver can access private members of Node
	// friend class SingleAgentSolver;
};


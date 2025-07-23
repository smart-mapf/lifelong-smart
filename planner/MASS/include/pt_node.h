// This file implement the PT Node
#pragma once
#include <iostream>
#include <fstream>
#include <stack>
#include <set>
#include <tuple>
#include <vector>
#include "common.h"
#include "instance.h"
#include "conflict.h"

const double EPSILON = 0.000001;

class PTNode{
public:
	PTNode() = default;
    PTNode(const std::shared_ptr<PTNode>& parent): plan(parent->plan), motion_solution(parent->motion_solution), all_agents_timed_path(parent->all_agents_timed_path),
                                                   solution_cost(parent->solution_cost), cost(parent->cost), depth(parent->depth+1),
                                                   makespan(parent->makespan), conflicts(parent->conflicts), parent(parent) {}
    PTNode(const vector<Path>&, std::vector<MotionInfo>& solution,
           std::vector<TimedPath>& timed_paths, const std::map<int, std::set<int> >&);
	// void writeToFile(Instance&, const string&);	
	void calculateCost();
	void calculateCost(std::shared_ptr<Instance> instance_ptr);

	void topologicalSortUtil(int, std::vector<bool>& visited, std::list<int>&);
	std::list<int> topologicalSort(int);
    std::pair<int, int> getFirstCollision(Instance&);
	void getRTP(std::set<int>&, int);
	void getRTFromP(ReservationTable&, std::set<int>&);
    void insertPathToRT(ReservationTable& rt, int agent_id);
	void printPriorityMap();
	void printPlan();
    int checkValid(ReservationTable &rt, int agent_id);
    bool CheckCollision(int agent_first, int agent_second);
    bool checkSolution(Instance& instance);
    void printConflicts();


public:
    vector<Path> plan; //plan is vector of paths	size of vector = size of vehicles
    std::vector<MotionInfo> motion_solution;
    std::vector<double> solution_cost;
    std::vector<TimedPath> all_agents_timed_path;
    double cost = 0.0;
    int depth = 0;
    double makespan = 0;
    // conflicts in the current node
    list<shared_ptr<Conflict> > conflicts;
    shared_ptr<Conflict> conflict;
    shared_ptr<Conflict> curr_conflict;

    std::pair<std::shared_ptr<PTNode>, std::shared_ptr<PTNode>> children;
    Constraint constraint;

    std::map<int, std::set<int>> priority; //map of index with set of lower priority
    std::shared_ptr<PTNode> parent;
};


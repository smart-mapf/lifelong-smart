#pragma once
#include "backup_planners/BasicGraph.h"
#include "backup_planners/States.h"

class ReservationTable {
public:
    size_t map_size;
    int num_of_agents;
    int k_robust;
    int window;
    bool use_cat;  // use conflict avoidance table
    bool hold_endpoints = false;

    bool prioritize_start;
    double runtime;

    void clear() {
        sit.clear();
        ct.clear();
        cat.clear();
    }
    void copy(const ReservationTable& other) {
        sit = other.sit;
        ct = other.ct;
        cat = other.cat;
    }
    void build(const vector<Path*>& paths,
               const list<tuple<int, int, int> >& initial_constraints,
               const unordered_set<int>& high_priority_agents,
               int current_agent, int start_location);
    void build(const vector<Path>& paths,
               const list<tuple<int, int, int> >& initial_constraints,
               int current_agent);
    void build(const vector<Path*>& paths,
               const list<tuple<int, int, int> >& initial_constraints,
               const list<Constraint>& constraints, int current_agent);
    // insert the path to the constraint table
    void insertPath2CT(const Path& path);
    void print() const;
    void printCT(size_t location) const;

    // functions for SIPP
    // Return the overlapping safe interval between current node and the
    // neighboring node.
    list<Interval> getSafeIntervals(int location, int lower_bound,
                                    int upper_bound);
    list<Interval> getSafeIntervals(int from, int to, int lower_bound,
                                    int upper_bound);

    // The earliest time that the agent can stay at the location forever
    int getHoldingTimeFromSIT(int location);
    Interval getFirstSafeInterval(int location);
    bool findSafeInterval(Interval& interval, int location, int t_min);

    // Check if the agent can wait at the location during the given time
    // interval
    bool canWaitUntil(int location, int desired_t_min, int desired_t_max);

    // functions for state-time A*
    bool isConstrained(int curr_id, int next_id, int next_timestep) const;
    bool isConflicting(int curr_id, int next_id, int next_timestep) const;
    int getHoldingTimeFromCT(int location) const;
    set<int> getConstrainedTimesteps(int location) const;

    ReservationTable(const BasicGraph& G) : G(G) {
    }

private:
    const BasicGraph& G;
    // Constraint Table (CT), stores the constraints, aka the time ranges that
    // the agents CANNOT traverse.
    // location/edge -> time range
    unordered_map<size_t, list<pair<int, int> > > ct;

    // Conflict Avoidance Table (CAT): use path of ALL other agents to check
    // for collisions. Used to count the number of conflicts of each agent's
    // path with other agents.
    //  (timestep, location) ->  have conflicts or not
    vector<vector<bool> > cat;

    // Safe Interval Table (SIT): time ranges the agents CAN traverse, aka the
    // time ranges not in CT, `num_of_collisions` is obtained from CAT.
    // location/edge -> [t_min, t_max), num_of_collisions
    unordered_map<size_t, list<Interval> > sit;

    void updateSIT(size_t location);  // update SIT at the given location
    void mergeIntervals(
        list<Interval>& intervals) const;  // merge successive safe intervals
                                           // with the same number of conflicts.

    void insertConstraint2SIT(int location, int t_min, int t_max);
    void insertSoftConstraint2SIT(int location, int t_min, int t_max);
    void insertConstraints4starts(const vector<Path*>& paths, int current_agent,
                                  int start_location);
    void insertPath2CAT(
        const Path& path);  //  insert the path to the conflict avoidance table
    void addInitialConstraints(
        const list<tuple<int, int, int> >& initial_constraints,
        int current_agent);
    inline int getEdgeIndex(int from, int to) const {
        return (from + 1) * map_size + to;
    }
    inline pair<int, int> getEdge(int index) const {
        return make_pair(index / map_size - 1, index % map_size);
    }
};
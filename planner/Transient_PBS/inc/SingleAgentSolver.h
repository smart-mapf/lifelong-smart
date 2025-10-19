#pragma once
#include "ConstraintTable.h"
#include "Instance.h"

class LLNode  // low-level node
{
public:
    int location;
    double g_val;
    double h_val = 0;
    LLNode* parent;
    int timestep = 0;
    int num_of_conflicts = 0;
    bool in_openlist = false;
    // the action is to wait at the goal vertex or not. This is used for
    // >lenghth constraints
    bool wait_at_goal = false;
    bool is_goal = false;

    // Whether any of this node's ancestors have visited the goal. Used by
    // Transient MAPF.
    bool visited_goal = false;

    // Return true if any of this node's ancestors have visited the goal.
    bool ancestorVisitedGoal() const {
        if (this->parent == nullptr)
            return false;
        auto curr = this->parent;
        while (curr != nullptr) {
            if (curr->visited_goal)
                return true;
            curr = curr->parent;
        }
        return false;
    }

    // the following is used to comapre nodes in the OPEN list
    struct compare_node {
        // returns true if n1 > n2 (note -- this gives us *min*-heap).
        bool operator()(const LLNode* n1, const LLNode* n2) const {
            // If n1 has visited goal, and n2 has not, prefer n1
            if (n1->visited_goal && !n2->visited_goal)
                return true;
            // If n2 has visited goal, and n1 has not, prefer n2
            if (n2->visited_goal && !n1->visited_goal)
                return false;

            // Both has visited goal, prefer the one with smaller
            // makespan
            if (n1->visited_goal && n2->visited_goal) {
                return n1->timestep >= n2->timestep;
            }

            // Both has not visited goal, use normal comparison
            if (n1->g_val + n1->h_val == n2->g_val + n2->h_val) {
                if (n1->num_of_conflicts == n2->num_of_conflicts) {
                    if (n1->h_val == n2->h_val) {
                        // return rand() % 2 == 0;   // break ties randomly
                        return false;
                    }
                    // break ties towards smaller h_vals
                    // (closer to goal location)
                    return n1->h_val >= n2->h_val;
                }
                // n1 > n2 if it has more conflicts
                return n1->num_of_conflicts >= n2->num_of_conflicts;
            }
            return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
        }
    };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val,
        // and then highest g-val)

    // the following is used to compare nodes in the FOCAL list
    struct secondary_compare_node {
        // returns true if n1 > n2
        bool operator()(const LLNode* n1, const LLNode* n2) const {
            if (n1->num_of_conflicts == n2->num_of_conflicts) {
                if (n1->g_val + n1->h_val == n2->g_val + n2->h_val) {
                    if (n1->h_val == n2->h_val) {
                        // return rand() % 2 == 0;   // break ties randomly
                        return false;
                    }
                    // break ties towards smaller h_vals
                    // (closer to goal location)
                    return n1->h_val >= n2->h_val;
                }
                // break ties towards smaller f_vals
                // (prefer shorter solutions)
                return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
            }
            // n1 > n2 if it has more conflicts
            return n1->num_of_conflicts >= n2->num_of_conflicts;
        }
    };  // used by FOCAL (heap) to compare nodes (top of the heap has min
        // number-of-conflicts)

    LLNode()
        : location(0),
          g_val(0),
          h_val(0),
          parent(nullptr),
          timestep(0),
          num_of_conflicts(0),
          in_openlist(false),
          wait_at_goal(false),
          is_goal(false),
          visited_goal(false) {
    }

    LLNode(int location, int g_val, int h_val, LLNode* parent, int timestep,
           bool visited_goal = false, int num_of_conflicts = 0,
           bool in_openlist = false)
        : location(location),
          g_val(g_val),
          h_val(h_val),
          parent(parent),
          timestep(timestep),
          num_of_conflicts(num_of_conflicts),
          in_openlist(in_openlist),
          wait_at_goal(false),
          visited_goal(visited_goal) {
    }

    inline double getFVal() const {
        return g_val + h_val;
    }
    void copy(const LLNode& other) {
        location = other.location;
        g_val = other.g_val;
        h_val = other.h_val;
        parent = other.parent;
        timestep = other.timestep;
        num_of_conflicts = other.num_of_conflicts;
        wait_at_goal = other.wait_at_goal;
        is_goal = other.is_goal;
        visited_goal = other.visited_goal;
    }
};

class SingleAgentSolver {
public:
    uint64_t num_expanded = 0;
    uint64_t num_generated = 0;

    // runtime of building constraint table
    double runtime_build_CT = 0;
    // runtime of building conflict avoidance table
    double runtime_build_CAT = 0;

    int start_location;
    int goal_location;

    const Instance& instance;

    virtual Path findOptimalPath(const set<int>& higher_agents,
                                 const vector<Path*>& paths, int agent) = 0;
    virtual string getName() const = 0;

    list<int> getNextLocations(
        int curr) const;  // including itself and its neighbors
    list<int> getNeighbors(int curr) const {
        return instance.graph->getNeighbors(curr);
    }

    SingleAgentSolver(const Instance& instance, int agent, int screen = 0)
        : instance(instance),  // agent(agent),
          start_location(instance.start_locations[agent]),
          goal_location(instance.goal_locations[agent].loc),
          screen(screen) {
    }

    virtual ~SingleAgentSolver() {
    }

protected:
    double min_f_val;  // minimal f value in OPEN
    double w = 1;      // suboptimal bound
    int screen = 0;
};

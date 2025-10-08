#pragma once
#include "utils/States.h"
#include "utils/common.h"

class StateAStarNode {
public:
    State state;
    double g_val;
    double h_val;
    StateAStarNode* parent;
    int conflicts;
    int depth;
    bool in_openlist;
    int goal_id;  // the id of its current goal.

    // the following is used to comapre nodes in the OPEN list
    struct compare_node {
        // returns true if n1 > n2 (note -- this gives us *min*-heap).
        bool operator()(const StateAStarNode* n1,
                        const StateAStarNode* n2) const {
            if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                return rand() % 2 == 0;  // break ties randomly
            return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
        }
    };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val,
        // and then highest g-val)

    // the following is used to comapre nodes in the FOCAL list
    struct secondary_compare_node {
        // returns true if n1 > n2
        bool operator()(const StateAStarNode* n1,
                        const StateAStarNode* n2) const {
            if (n1->conflicts == n2->conflicts) {
                // break ties towards larger g_vals
                return n1->g_val <= n2->g_val;
            }
            // n1 > n2 if it has more conflicts
            return n1->conflicts >= n2->conflicts;
        }
    };  // used by FOCAL (heap) to compare nodes (top of the heap has min
        // number-of-conflicts)

    // define a typedefs for handles to the heaps (allow up to quickly update a
    // node in the heap)
    fibonacci_heap<StateAStarNode*,
                   compare<StateAStarNode::compare_node> >::handle_type
        open_handle;
    fibonacci_heap<StateAStarNode*,
                   compare<StateAStarNode::secondary_compare_node> >::
        handle_type focal_handle;

    StateAStarNode()
        : g_val(0),
          h_val(0),
          parent(nullptr),
          conflicts(0),
          depth(0),
          in_openlist(false),
          goal_id(0) {
    }
    StateAStarNode(const State& state, double g_val, double h_val,
                   StateAStarNode* parent, int conflicts)
        : state(state),
          g_val(g_val),
          h_val(h_val),
          parent(parent),
          conflicts(conflicts),
          in_openlist(false) {
        if (parent != nullptr) {
            depth = parent->depth + 1;
            goal_id = parent->goal_id;
        } else {
            depth = 0;
            goal_id = 0;
        }
    }

    /*AStarNode(const AStarNode& other)
    {
        state = other.state;
        g_val = other.g_val;
        h_val = other.h_val;
        parent = other.parent;
        in_openlist = other.in_openlist;
        open_handle = other.open_handle;
        focal_handle = other.focal_handle;
        conflicts = other.conflicts;
        depth = other.depth;
        goal_id = other.goal_id;
    }*/

    inline double getFVal() const {
        return g_val + h_val;
    }

    // The following is used to  check whether two nodes are equal
    // we say that two nodes are equal iff
    // both agree on the state and the goal id
    struct EqNode {
        bool operator()(const StateAStarNode* n1,
                        const StateAStarNode* n2) const {
            return (n1 == n2) || (n1 && n2 && n1->state == n2->state &&
                                  n1->goal_id == n2->goal_id);
        }
    };

    // The following is used to generate the hash value of a node
    struct Hasher {
        std::size_t operator()(const StateAStarNode* n) const {
            return State::Hasher()(n->state);
        }
    };
};

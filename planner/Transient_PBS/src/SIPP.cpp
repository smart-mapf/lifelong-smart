#include "SIPP.h"

void SIPP::updatePath(const LLNode* goal, vector<PathEntry>& path) {
    // num_collisions = goal->num_of_conflicts;
    path.resize(goal->timestep + 1);
    // num_of_conflicts = goal->num_of_conflicts;

    const auto* curr = goal;
    while (curr->parent != nullptr)  // non-root node
    {
        const auto* prev = curr->parent;
        int t = prev->timestep + 1;
        while (t < curr->timestep) {
            path[t].location = prev->location;  // wait at prev location
            t++;
        }
        // move to curr location
        path[curr->timestep].location = curr->location;
        curr = prev;
    }
    assert(curr->timestep == 0);
    path[0].location = curr->location;
    path.back().sum_of_costs = goal->g_val;
}

Path SIPP::findOptimalPath(const set<int>& higher_agents,
                           const vector<Path*>& paths, int agent) {
    reset();

    // build constraint table
    auto t = clock();
    ConstraintTable constraint_table(instance.graph->num_of_cols,
                                     instance.graph->map_size);
    for (int a : higher_agents) {
        constraint_table.insert2CT(*paths[a]);
    }
    runtime_build_CT = (double)(clock() - t) / CLOCKS_PER_SEC;

    // double holding_time =
    // static_cast<double>(constraint_table.getHoldingTime(
    //     goal_location, constraint_table.length_min));

    t = clock();
    constraint_table.insert2CAT(agent, paths);
    runtime_build_CAT = (double)(clock() - t) / CLOCKS_PER_SEC;

    // build reservation table
    ReservationTable reservation_table(constraint_table, goal_location);

    Path path;
    num_expanded = 0;
    num_generated = 0;
    Interval interval =
        reservation_table.get_first_safe_interval(start_location);
    if (get<0>(interval) > 0)
        return path;

    // generate start and add it to the OPEN list
    auto start = new SIPPNode(
        start_location, 0,
        instance.graph->heuristics.at(goal_location)[start_location], nullptr,
        0, get<1>(interval), get<1>(interval), get<2>(interval),
        get<2>(interval));
    // min_f_val = max(holding_time, start->getFVal());
    min_f_val = start->getFVal();
    pushNodeToOpen(start);

    while (!open_list.empty()) {
        auto curr = open_list.top();
        open_list.pop();
        curr->in_openlist = false;
        num_expanded++;

        // For transient MAPF, we stop the search if any of the ancestors of the
        // current node has visited the goal and if the agent can hold here
        // forever.

        // spdlog::info("Expanding node at location ({},{}) at timestep {}, "
        //              "visited goal: {}",
        //              instance.graph->getRowCoordinate(curr->location),
        //              instance.graph->getColCoordinate(curr->location),
        //              curr->timestep, curr->visited_goal);

        double holding_time = constraint_table.getHoldingTime(
            curr->location, constraint_table.length_min);

        if (curr->visited_goal && curr->timestep >= holding_time) {
            // spdlog::info("Agent {}'s ancestor has arrived at goal ({},{}), "
            //              "current loc ({}, {}) at, timestep {}, ",
            //              agent,
            //              instance.graph->getRowCoordinate(goal_location),
            //              instance.graph->getColCoordinate(goal_location),
            //              instance.graph->getRowCoordinate(curr->location),
            //              instance.graph->getColCoordinate(curr->location),
            //              curr->timestep);
            updatePath(curr, path);
            break;
        }

        // Otherwise, we continue the search.
        // Remember whether we have visited the goal location.
        bool visited_goal_loc = false;

        // We arrived at the goal location, but we cannot stay because of target
        // conflicts.
        if (curr->location == goal_location &&  // arrive at the goal location
            !curr->wait_at_goal)                // not wait at the goal location
        {
            // We need to do transient MAPF expansion.
            visited_goal_loc = true;
            // spdlog::info(
            //     "Agent {} arrived at goal location ({},{}) at timestep {}",
            //     agent, instance.graph->getRowCoordinate(goal_location),
            //     instance.graph->getColCoordinate(goal_location),
            //     curr->timestep);
        }

        // We set `visited_goal` of the successor node to true if we
        // have visited goal in curr node or if the curr node has the
        // flag to true.
        visited_goal_loc = visited_goal_loc || curr->visited_goal;

        // move to neighboring locations
        for (int next_location : instance.graph->getNeighbors(curr->location)) {
            for (auto& i : reservation_table.get_safe_intervals(
                     curr->location, next_location, curr->timestep + 1,
                     curr->high_expansion + 1)) {
                // Check if the edge if blocked.
                if (!instance.graph->validMove(curr->location, next_location))
                    continue;

                int next_high_generation, next_timestep, next_high_expansion;
                bool next_v_collision, next_e_collision;
                tie(next_high_generation, next_timestep, next_high_expansion,
                    next_v_collision, next_e_collision) = i;

                // Compute g and h
                double next_g_val = 0, next_h_val = 0;
                // If we have not visited the goal location, we compute g and h
                // normally.
                if (!visited_goal_loc) {
                    // compute cost to next_id via curr node
                    int wait_time = next_timestep - curr->timestep - 1;
                    // cost of waiting at the current location
                    double wait_cost = instance.graph->getWeight(
                        curr->location, curr->location);
                    double move_cost = instance.graph->getWeight(curr->location,
                                                                 next_location);
                    // new g val considers the wait cost and movement cost
                    next_g_val =
                        curr->g_val + wait_time * wait_cost + move_cost;
                    next_h_val = instance.graph->heuristics.at(
                        goal_location)[next_location];
                }
                // If any ancestors has visited the goal. We use the same g and
                // h as the parent.
                else {
                    next_g_val = curr->g_val;
                    next_h_val = curr->h_val;
                }

                // spdlog::info(
                //     "Generate successor at location ({},{}) at timestep {} "
                //     "with interval [{},{}), v_collision: {}, e_collision: "
                //     "{}, g_val: {}, h_val: {}",
                //     instance.graph->getRowCoordinate(next_location),
                //     instance.graph->getColCoordinate(next_location),
                //     next_timestep, next_high_generation, next_high_expansion,
                //     next_v_collision, next_e_collision, next_g_val,
                //     next_h_val);

                // Compute the number of conflicts.
                int next_conflicts =
                    curr->num_of_conflicts +
                    (int)curr->collision_v *
                        max(next_timestep - curr->timestep - 1, 0) +
                    +(int)next_v_collision + (int)next_e_collision;
                auto next = new SIPPNode(
                    next_location, next_g_val, next_h_val, curr, next_timestep,
                    next_high_generation, next_high_expansion, next_v_collision,
                    next_conflicts, visited_goal_loc);
                if (dominanceCheck(next))
                    pushNodeToOpen(next);
                else
                    delete next;
            }
        }  // end for loop that generates successors

        // wait at the current location
        int exp_duration =
            instance.graph->d_heuristics.at(goal_location)[curr->location];
        if (curr->high_expansion == curr->high_generation and
            reservation_table.find_safe_interval(interval, curr->location,
                                                 curr->high_expansion) and
            get<0>(interval) + exp_duration <=
                reservation_table.constraint_table.length_max) {
            auto next_timestep = get<0>(interval);
            // int next_h_val = max(curr->h_val, curr->getFVal() -
            // next_timestep);  // path max
            // we do not change h_val when waiting
            int next_h_val = curr->h_val;
            auto next_collisions =
                curr->num_of_conflicts +
                (int)curr->collision_v *
                    max(next_timestep - curr->timestep - 1, 0)  // wait time
                + (int)get<2>(interval);
            auto next = new SIPPNode(curr->location, next_timestep, next_h_val,
                                     curr, next_timestep, get<1>(interval),
                                     get<1>(interval), get<2>(interval),
                                     next_collisions, visited_goal_loc);
            if (curr->location == goal_location)
                next->wait_at_goal = true;
            if (dominanceCheck(next))
                pushNodeToOpen(next);
            else
                delete next;
        }
    }  // end while loop

    // no path found
    releaseNodes();
    return path;
}
void SIPP::updateFocalList() {
    auto open_head = open_list.top();
    if (open_head->getFVal() > min_f_val) {
        int new_min_f_val = (int)open_head->getFVal();
        for (auto n : open_list) {
            if (n->getFVal() > w * min_f_val &&
                n->getFVal() <= w * new_min_f_val)
                n->focal_handle = focal_list.push(n);
        }
        min_f_val = new_min_f_val;
    }
}
inline void SIPP::pushNodeToOpen(SIPPNode* node) {
    num_generated++;
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    allNodes_table[node].push_back(node);
}
inline void SIPP::pushNodeToOpenAndFocal(SIPPNode* node) {
    num_generated++;
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    if (node->getFVal() <= w * min_f_val)
        node->focal_handle = focal_list.push(node);
    allNodes_table[node].push_back(node);
}
inline void SIPP::pushNodeToFocal(SIPPNode* node) {
    num_generated++;
    allNodes_table[node].push_back(node);
    node->in_openlist = true;
    node->focal_handle =
        focal_list.push(node);  // we only use focal list; no open list is used
}
inline void SIPP::eraseNodeFromLists(SIPPNode* node) {
    if (open_list.empty()) {  // we only have focal list
        focal_list.erase(node->focal_handle);
    } else if (focal_list.empty()) {  // we only have open list
        open_list.erase(node->open_handle);
    } else {  // we have both open and focal
        open_list.erase(node->open_handle);
        if (node->getFVal() <= w * min_f_val)
            focal_list.erase(node->focal_handle);
    }
}
void SIPP::releaseNodes() {
    open_list.clear();
    focal_list.clear();
    for (auto& node_list : allNodes_table)
        for (auto n : node_list.second)
            delete n;
    allNodes_table.clear();
    for (auto n : useless_nodes)
        delete n;
    useless_nodes.clear();
}

// return true iff we the new node is not dominated by any old node
bool SIPP::dominanceCheck(SIPPNode* new_node) {
    auto ptr = allNodes_table.find(new_node);
    if (ptr == allNodes_table.end())
        return true;
    for (auto& old_node : ptr->second) {
        // the new node is dominated by the old node
        if (old_node->timestep <= new_node->timestep and
            old_node->num_of_conflicts <= new_node->num_of_conflicts) {
            return false;
        }
        // the old node is dominated by the new node, delete the old node
        else if (old_node->timestep >= new_node->timestep and
                 old_node->num_of_conflicts >= new_node->num_of_conflicts) {
            // the old node has not been expanded yet
            // delete it from open and/or focal lists
            if (old_node->in_openlist)
                eraseNodeFromLists(old_node);
            useless_nodes.push_back(old_node);
            ptr->second.remove(old_node);
            // this is because we later will increase num_generated when we
            // insert the new node into lists.
            num_generated--;
            return true;
        }
        // intervals overlap --> we need to split the node to make them disjoint
        else if (old_node->timestep < new_node->high_expansion and
                 new_node->timestep < old_node->high_expansion) {
            if (old_node->timestep <= new_node->timestep) {
                assert(old_node->num_of_conflicts > new_node->num_of_conflicts);
                old_node->high_expansion = new_node->timestep;
            } else  // i.e., old_node->timestep > new_node->timestep
            {
                assert(old_node->num_of_conflicts <=
                       new_node->num_of_conflicts);
                new_node->high_expansion = old_node->timestep;
            }
        }
    }
    return true;
}
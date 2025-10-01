#include "backup_planners/SIPP.h"

Path SIPP::updatePath(const BasicGraph& G, SIPPNode* goal) {
    Path path(goal->state.timestep + 1);
    path_cost = goal->getFVal();
    num_of_conf = goal->conflicts;

    SIPPNode* curr = goal;
    while (true) {
        // Handle tasking wait
        if (curr->tasking_wait > 0) {
            // Insert `curr->tasking_wait` wait actions
            // Currently it's very inefficient cuz we use `insert()` function of
            // std::vector. Maybe optimize a bit in the future.
            int lb_t = curr->state.timestep - curr->tasking_wait + 1;
            int up_t = curr->state.timestep;
            // auto it = path.begin();
            for (int t = lb_t; t <= up_t; t++) {
                path[t] = State(curr->state.location, t,
                                curr->state.orientation, true);
            }
            // Revert the timestep of the current node so that the agent does
            // not wait at the parent node
            curr->state.timestep -= curr->tasking_wait;
            curr->tasking_wait = 0;
        }

        if (curr->parent == nullptr)  // root node
        {
            int t = curr->state.timestep;
            path[t] = curr->state;

            if (curr->state.is_tasking_wait && curr->state.is_rotating) {
                cout << "Cannot wait for both tasking and rotating" << endl;
                exit(0);
            }

            if (curr->state.is_tasking_wait) {
                // t is larger than 0, meaning that we need some additional wait
                while (t > 0) {
                    t--;
                    path[t] = State(curr->state.location, t,
                                    curr->state.orientation, true);
                }
            }

            if (curr->state.is_rotating) {
                // t is larger than 0, meaning that we need some additional wait
                while (t > 0) {
                    t--;
                    path[t] = State(curr->state.location, t,
                                    curr->state.orientation, false, true);
                }
            }

            t--;
            for (; t >= 0; t--) {
                path[t] = State(-1, -1);  // dummy start states
            }
            break;
        } else {
            SIPPNode* prev = curr->parent;
            int degree = G.get_rotate_degree(prev->state.orientation,
                                             curr->state.orientation);
            int t = prev->state.timestep + 1;
            if (degree == 1)  // turn right or turn left
            {
                path[t] = State(prev->state.location, t,
                                curr->state.orientation, false, true);
                t++;
                // Add rotational waiting states
                for (int i = 0; i < this->rotation_time - 1; i++) {
                    path[t] = State(prev->state.location, t,
                                    curr->state.orientation, false, true);
                    t++;
                }
            } else if (degree == 2)  // turn back
            {
                // turn right
                path[t] = State(prev->state.location, t,
                                (prev->state.orientation + 1) % 4, false, true);
                t++;
                // Add rotational waiting states
                for (int i = 0; i < this->rotation_time - 1; i++) {
                    path[t] =
                        State(prev->state.location, t,
                              (prev->state.orientation + 1) % 4, false, true);
                    t++;
                }
                // turn right
                path[t] = State(prev->state.location, t,
                                curr->state.orientation, false, true);
                t++;
                // Add rotational waiting states
                for (int i = 0; i < this->rotation_time - 1; i++) {
                    path[t] = State(prev->state.location, t,
                                    curr->state.orientation, false, true);
                    t++;
                }
            }
            while (t < curr->state.timestep) {
                // cout << "Adding wait at " << t << endl;
                path[t] =
                    State(prev->state.location, t,
                          curr->state.orientation);  // wait at prev location
                t++;
            }
            // Since we added the special expansion that contains only
            // rotation, we may not add move action
            if (path[curr->state.timestep].location < 0) {
                // cout << "Adding moving at " << curr->state.timestep << endl;
                path[curr->state.timestep] =
                    State(curr->state.location, curr->state.timestep,
                          curr->state.orientation);  // move to current location
            }
            curr = prev;
        }
    }
    return path;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// return true if a path found (and updates vector<int> path) or false if no
// path exists after max_timestep, switch from time-space A* search to normal A*
// search
Path SIPP::run(const BasicGraph& G, const State& start,
               const vector<Task>& goal_location, ReservationTable& rt,
               // Number of timestep that the agent has waited before entering
               // the search
               const int agent_waited_time) {
    num_expanded = 0;
    num_generated = 0;
    runtime = 0;
    clock_t t = std::clock();
    double h_val = compute_h_value(G, start, 0, goal_location);
    if (h_val > INT_MAX) {
        // cout << "The start and goal locations are disconnected!" << endl;
        spdlog::error("The start and goal locations are disconnected!");
        return Path();
    }
    Interval interval = rt.getFirstSafeInterval(start.location);

    if (std::get<0>(interval) == 0) {
        auto node = new SIPPNode(start, 0, h_val, interval, nullptr, 0);
        num_generated++;
        node->open_handle = open_list.push(node);
        node->in_openlist = true;
        allNodes_table.insert(node);
        min_f_val = node->getFVal();
        focal_bound = min_f_val * suboptimal_bound;
        node->focal_handle = focal_list.push(node);
    } else if (prioritize_start)  // the agent has the highest priority at its
                                  // start location
    {
        // This is correct only when k_robust <= 1. Otherwise, agents might not
        // be able to  wait at its start locations due to initial constraints
        // caused by the previous actions of other agents.
        Interval interval = make_tuple(0, INTERVAL_MAX, 0);
        auto node = new SIPPNode(start, 0, h_val, interval, nullptr, 0);
        num_generated++;
        node->open_handle = open_list.push(node);
        node->in_openlist = true;
        allNodes_table.insert(node);
        min_f_val = node->getFVal();
        focal_bound = min_f_val;
        node->focal_handle = focal_list.push(node);
    }
    int earliest_holding_time = 0;
    if (hold_endpoints)
        earliest_holding_time =
            rt.getHoldingTimeFromSIT(goal_location.back().location);
    while (!focal_list.empty()) {
        // std::cout << "sipp, pop node" <<std::endl;
        // spdlog::info("SIPP: Popping node from focal list...");
        SIPPNode* curr = focal_list.top();
        focal_list.pop();
        open_list.erase(curr->open_handle);
        curr->in_openlist = false;
        num_expanded++;
        // cout << "node expanded " << num_expanded << endl;
        // spdlog::info("SIPP: Popped node");

        // Make sure agents wait enough time at each goal.
        // For manufactual system, consecutive goals could be the same.
        // In that case, we simply increment the tasking wait time to generate
        // the final path.
        auto curr_goal = goal_location[curr->goal_id];
        if (!this->consider_task_wait) {
            // update goal id
            if (curr->state.location == curr_goal.location &&
                // We do not consider goal orientation if it is the given as -1
                // otherwise the goal orientation must match.
                (curr_goal.orientation == -1 ||
                 curr->state.orientation == curr_goal.orientation) &&
                // reach the goal location after its release time
                curr->state.timestep >= curr_goal.hold_time) {
                curr->goal_id++;
                if (curr->goal_id == (int)goal_location.size() &&
                    earliest_holding_time > curr->state.timestep)
                    curr->goal_id--;
            }
        } else {
            // spdlog::info("SIPP: Considering task wait time for goal id: {}",
            //             curr->goal_id);
            while (curr->state.location == curr_goal.location &&
                   // We do not consider goal orientation if it is the given as
                   // -1 otherwise the goal orientation must match.
                   (curr_goal.orientation == -1 ||
                    curr->state.orientation == curr_goal.orientation) &&
                   // reach the goal location after its release time
                   curr->state.timestep >= curr_goal.hold_time &&
                   curr->goal_id < (int)goal_location.size()) {
                // update goal id
                curr->goal_id++;
                if (curr->goal_id == (int)goal_location.size() &&
                    earliest_holding_time > curr->state.timestep) {
                    curr->goal_id--;
                } else {
                    // Update tasking wait time of the node.
                    // Make sure it is safe to do so by checking the safe
                    // interval of the goal location
                    int wait_start_t = curr->state.timestep;
                    int wait_until_t =
                        curr->state.timestep + curr_goal.task_wait_time;
                    bool can_wait = rt.canWaitUntil(curr->state.location,
                                                    wait_start_t, wait_until_t);
                    if (can_wait) {
                        curr->state.timestep += curr_goal.task_wait_time;
                        curr->tasking_wait += curr_goal.task_wait_time;
                        curr_goal = goal_location[curr->goal_id];
                    } else {
                        curr->goal_id--;
                        break;
                    }
                }
            }
        }

        // check if the popped node is a goal
        if (curr->goal_id == (int)goal_location.size()) {
            // cout << "is goal: " << curr->goal_id << " "
            //      << (int)goal_location.size() << endl;
            // spdlog::info("SIPP: Found goal node with goal id: {}",
            // curr->goal_id);
            Path path = updatePath(G, curr);
            releaseClosedListNodes();
            open_list.clear();
            focal_list.clear();
            runtime = (std::clock() - t) * 1.0 / CLOCKS_PER_SEC;
            return path;
        }

        // expand the nodes
        // std::cout << "sipp, expand node" <<std::endl;
        for (int orientation = 0; orientation < 4; orientation++)  // move
        {
            // the edge is blocked
            if (!G.valid_move(curr->state.location, orientation))
                continue;
            int degree;
            if (curr->state.orientation < 0)
                degree = 0;
            else
                degree =
                    G.get_rotate_degree(curr->state.orientation, orientation);
            // don't have enough time to turn
            if (degree * this->rotation_time >
                std::get<1>(curr->interval) - curr->state.timestep)
                continue;
            int location = curr->state.location + G.move[orientation];
            int next_ori = -1;
            if (curr->state.orientation >= 0) {
                next_ori = G.get_direction(curr->state.location, location);
            }
            // std::cout << "compute h value, location ="<<location <<", goal id
            // = "<< curr->goal_id <<std::endl;
            double h_val = compute_h_value(G, State(location, 0, next_ori),
                                           curr->goal_id, goal_location);
            // std::cout << "h value = "<<h_val <<std::endl;
            // This vertex cannot reach the goal vertex
            if (h_val >= WEIGHT_MAX)
                continue;
            // degree * rotation_time: # timesteps to rotate
            // + 1: # timesteps to move
            int min_timestep =
                curr->state.timestep + degree * this->rotation_time + 1;
            for (auto interval : rt.getSafeIntervals(
                     curr->state.location, location, min_timestep,
                     std::get<1>(curr->interval) + 1)) {
                if (curr->state.orientation < 0)
                    generate_node(interval, curr, G, location, min_timestep, -1,
                                  h_val);
                else
                    generate_node(interval, curr, G, location, min_timestep,
                                  orientation, h_val);
            }

        }  // end for loop that generates successors

        // Special expansion for goal location: If we want the goal orientation
        // of the state to match the given goal, we need to generate
        // successors where only rotation is applied to the current state
        if (curr_goal.orientation != -1 &&
            curr->state.orientation != curr_goal.orientation &&
            curr->state.location == curr_goal.location &&
            curr->state.timestep >= curr_goal.hold_time &&
            this->rotation_time <=
                std::get<1>(curr->interval) - curr->state.timestep) {
            double h_val =
                compute_h_value(G, curr->state, curr->goal_id, goal_location);
            int degree = 1;
            int min_timestep =
                curr->state.timestep + degree * this->rotation_time;
            for (auto interval :
                 rt.getSafeIntervals(curr->state.location, min_timestep,
                                     std::get<1>(curr->interval) + 1)) {
                generate_node(interval, curr, G, curr->state.location,
                              min_timestep, (curr->state.orientation + 1) % 4,
                              h_val);
                generate_node(interval, curr, G, curr->state.location,
                              min_timestep, (curr->state.orientation + 3) % 4,
                              h_val);
            }
        }

        if (rt.use_cat)  // wait to the successive interval
        {
            int location = curr->state.location;
            int min_timestep = std::get<1>(curr->interval);
            int orientation = curr->state.orientation;
            Interval interval;
            bool found = rt.findSafeInterval(interval, location, min_timestep);
            if (found) {
                if (curr->state.orientation < 0) {
                    generate_node(interval, curr, G, location, min_timestep, -1,
                                  curr->h_val);
                } else {
                    generate_node(interval, curr, G, location, min_timestep,
                                  orientation, curr->h_val);
                    generate_node(interval, curr, G, location, min_timestep,
                                  (orientation + 1) % 4, curr->h_val);
                    generate_node(interval, curr, G, location, min_timestep,
                                  (orientation + 3) % 4, curr->h_val);
                    if (std::get<1>(curr->interval) - curr->state.timestep > 1)
                        generate_node(interval, curr, G, location, min_timestep,
                                      (orientation + 2) % 4, curr->h_val);
                }
            }
        }

        // update FOCAL if min f-val increased
        // std::cout << "whether focal" <<std::endl;
        if (open_list.empty())  // in case OPEN is empty, no path found
        {
            // the agent has the highest priority at its start location
            if (prioritize_start) {
                // This is correct only when k_robust <= 1. Otherwise, agents
                // might not be able to wait at its start locations due to
                // initial constraints caused by the previous actions of other
                // agents.
                Interval interval = rt.getFirstSafeInterval(start.location);
                Interval interval2 =
                    make_tuple(std::get<1>(interval), INTERVAL_MAX, 0);
                double h_val = compute_h_value(G, start, 0, goal_location);
                auto node2 =
                    new SIPPNode(start, 0, h_val, interval2, nullptr, 0);
                num_generated++;
                node2->open_handle = open_list.push(node2);
                node2->in_openlist = true;
                allNodes_table.insert(node2);
                min_f_val = node2->getFVal();
                focal_bound = min_f_val;
                node2->focal_handle = focal_list.push(node2);
            } else {
                break;
            }
        } else {
            SIPPNode* open_head = open_list.top();
            if (open_head->getFVal() > min_f_val) {
                double new_min_f_val = open_head->getFVal();
                double new_focal_bound = new_min_f_val * suboptimal_bound;
                for (SIPPNode* n : open_list) {
                    if (n->getFVal() > focal_bound &&
                        n->getFVal() <= new_focal_bound)
                        n->focal_handle = focal_list.push(n);
                }
                min_f_val = new_min_f_val;
                focal_bound = new_focal_bound;
            }
        }

    }  // end while loop

    // no path found
    // std::cout << "sipp, post process" <<std::endl;
    releaseClosedListNodes();
    open_list.clear();
    focal_list.clear();
    return Path();
}

/*void SIPP::generate_node(SIPPNode* curr, const SortationGrid& G,
                         int location, int timestep, int orientation, double
h_val)
{
    int wait_time = timestep - curr->state.timestep - 1; // inlcude rotate time
    double travel_time = 1;
    if (!travel_times.empty())
    {
        int dir = G.get_direction(curr->state.location, location);
        travel_time += travel_times[curr->state.location][dir];
    }
    double g_val = curr->g_val + travel_time * (wait_time *
G.get_weight(curr->state.location, curr->state.location)
                   + G.get_weight(curr->state.location, location));

    int conflicts = curr->conflicts;

    // generate (maybe temporary) node
    auto next = new SIPPNode(State(location, timestep, orientation),
                             g_val, h_val, Interval(window + 1, INTERVAL_MAX,
0), curr, conflicts);

    // try to retrieve it from the hash table
    auto it = allNodes_table.find(next);
    if (it != allNodes_table.end() && (*it)->state.timestep !=
next->state.timestep) { // arrive at the same interval at different timestep int
waiting_time = (*it)->state.timestep - next->state.timestep; double waiting_cost
= abs(G.get_weight(next->state.location, next->state.location) * waiting_time);
        double next_f_val = next->getFVal() + waiting_cost;
        if (waiting_time > 0 && next_f_val <= (*it)->getFVal())
        { // next arrives earlier with a smaller cost
            // so delete it
            // let the following update it with next
        }
        else if (waiting_time < 0 && next_f_val >= (*it)->getFVal())
        { // it arrives earlier with a smaller cost
            delete next; // so delete next
            return;
        }
        else // the later node arrives with a smaller cost, so they cannot be
regarded as the same state it = allNodes_table.end();
    }
    if (it == allNodes_table.end())
    {
        next->open_handle = open_list.push(next);
        next->in_openlist = true;
        num_generated++;
        if (next->getFVal() <= focal_bound)
            next->focal_handle = focal_list.push(next);
        allNodes_table.insert(next);
        return;
    }

    // update existing node if needed (only in the open_list)
    SIPPNode* existing_next = *it;
    double existing_f_val = existing_next->getFVal();

    if (existing_next->in_openlist)
    {  // if its in the open list
        if (existing_f_val > g_val + h_val ||
            (existing_f_val == g_val + h_val && existing_next->conflicts >
conflicts))
        {
            // if f-val decreased through this new path (or it remains the same
and there's less internal conflicts) bool add_to_focal = false;  // check if it
was above the focal bound before and now below (thus need to be inserted) bool
update_in_focal = false;  // check if it was inside the focal and needs to be
updated (because f-val changed) bool update_open = false; if ((g_val + h_val) <=
focal_bound) {  // if the new f-val qualify to be in FOCAL if (existing_f_val >
focal_bound) add_to_focal = true;  // and the previous f-val did not qualify to
be in FOCAL then add else update_in_focal = true;  // and the previous f-val did
qualify to be in FOCAL then update
            }
            if (existing_f_val > g_val + h_val)
                update_open = true;
            // update existing node
            existing_next->state = next->state;
            existing_next->g_val = g_val;
            existing_next->h_val = h_val;
            existing_next->parent = curr;
            existing_next->depth = next->depth;
            existing_next->conflicts = conflicts;
            // existing_next->move = next->move;

            if (update_open)
                open_list.increase(existing_next->open_handle);  // increase
because f-val improved if (add_to_focal) existing_next->focal_handle =
focal_list.push(existing_next); if (update_in_focal)
                focal_list.update(existing_next->focal_handle);  // should we do
update? yes, because number of conflicts may go up or down
        }
    }
    else
    {  // if its in the closed list (reopen)
        if (existing_f_val > g_val + h_val ||
            (existing_f_val == g_val + h_val && existing_next->conflicts >
conflicts))
        {
            // if f-val decreased through this new path (or it remains the same
and there's less internal conflicts) existing_next->state = next->state;
            existing_next->g_val = g_val;
            existing_next->h_val = h_val;
            existing_next->parent = curr;
            existing_next->depth = next->depth;
            existing_next->conflicts = conflicts;
            existing_next->open_handle = open_list.push(existing_next);
            existing_next->in_openlist = true;
            if (existing_f_val <= focal_bound)
                existing_next->focal_handle = focal_list.push(existing_next);
        }
    }  // end update a node in closed list

    delete(next);  // not needed anymore -- we already generated it before
}*/

void SIPP::generate_node(const Interval& interval, SIPPNode* curr,
                         const BasicGraph& G, int location, int min_timestep,
                         int orientation, double h_val) {
    int timestep = max(std::get<0>(interval), min_timestep);
    int wait_time = timestep - curr->state.timestep - 1;  // inlcude rotate time
    double g_val =
        curr->g_val +
        wait_time * G.get_weight(curr->state.location, curr->state.location) +
        G.get_weight(curr->state.location, location);

    int conflicts = std::get<2>(interval) + curr->conflicts;

    // generate (maybe temporary) node
    auto next = new SIPPNode(State(location, timestep, orientation), g_val,
                             h_val, interval, curr, conflicts);

    // try to retrieve it from the hash table
    auto it = allNodes_table.find(next);
    /*if (it != allNodes_table.end() && (*it)->state.timestep !=
    next->state.timestep) { // arrive at the same interval at different timestep
        int waiting_time = (*it)->state.timestep - next->state.timestep;
        double waiting_cost = abs(G.get_weight(next->state.location,
    next->state.location) * waiting_time); if (waiting_time > 0 &&
    next->getFVal() + waiting_cost <= (*it)->getFVal()) { // next arrives later
    with a smaller cost
            // let the following update it with next
        }
        else if (waiting_time < 0 && next->getFVal() >= (*it)->getFVal() +
    waiting_cost) { // next arrives earlier with a larger cost delete next; //
    so delete next return;
        }
        else // next arrives with a smaller cost, so they cannot be regarded as
    the same state it = allNodes_table.end(); // TODO: fix this bug! When later
    inserting this node to allNodes_table, it will not override the previous
    node.
    }*/
    if (it == allNodes_table.end()) {
        next->open_handle = open_list.push(next);
        next->in_openlist = true;
        num_generated++;
        if (next->getFVal() <= focal_bound)
            next->focal_handle = focal_list.push(next);
        allNodes_table.insert(next);
        return;
    }

    // update existing node if needed (only in the open_list)
    SIPPNode* existing_next = *it;
    double existing_f_val = existing_next->getFVal();

    if (existing_next->in_openlist) {  // if its in the open list
        if (existing_f_val > g_val + h_val ||
            (existing_f_val == g_val + h_val &&
             existing_next->conflicts > conflicts)) {
            // if f-val decreased through this new path (or it remains the same
            // and there's less internal conflicts)
            // check if it was above the focal bound before and now
            // below (thus need to be inserted)
            bool add_to_focal = false;
            // check if it was inside the focal and needs to be
            // updated (because f-val changed)
            bool update_in_focal = false;
            bool update_open = false;
            // if the new f-val qualify to be in FOCAL
            if ((g_val + h_val) <= focal_bound) {
                // and the previous f-val did not qualify to be in FOCAL then
                // add
                if (existing_f_val > focal_bound)
                    add_to_focal = true;
                // and the previous f-val did qualify to be in FOCAL then update
                else
                    update_in_focal = true;
            }
            if (existing_f_val > g_val + h_val)
                update_open = true;
            // update existing node
            existing_next->state = next->state;
            existing_next->g_val = g_val;
            existing_next->h_val = h_val;
            existing_next->parent = curr;
            existing_next->depth = next->depth;
            existing_next->conflicts = conflicts;
            // existing_next->move = next->move;

            // increase because f-val improved
            if (update_open)
                open_list.increase(existing_next->open_handle);
            if (add_to_focal)
                existing_next->focal_handle = focal_list.push(existing_next);
            // should we do update? yes, because number of conflicts may go up
            // or down
            if (update_in_focal)
                focal_list.update(existing_next->focal_handle);
        }
    } else {  // if its in the closed list (reopen)
        if (existing_f_val > g_val + h_val ||
            (existing_f_val == g_val + h_val &&
             existing_next->conflicts > conflicts)) {
            // if f-val decreased through this new path (or it remains the same
            // and there's less internal conflicts)
            existing_next->state = next->state;
            existing_next->g_val = g_val;
            existing_next->h_val = h_val;
            existing_next->parent = curr;
            existing_next->depth = next->depth;
            existing_next->conflicts = conflicts;
            existing_next->open_handle = open_list.push(existing_next);
            existing_next->in_openlist = true;
            if (existing_f_val <= focal_bound)
                existing_next->focal_handle = focal_list.push(existing_next);
        }
    }  // end update a node in closed list

    delete (next);  // not needed anymore -- we already generated it before
}

inline void SIPP::releaseClosedListNodes() {
    for (auto it = allNodes_table.begin(); it != allNodes_table.end(); it++)
        delete (*it);
    allNodes_table.clear();
}

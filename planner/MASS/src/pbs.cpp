/**
 * Implementation of the PBS class
 *
 */
#include "pbs.h"

/**
 * Class constructor for PBS
 *
 * @param instance the pointer of instance that is derived from the input files
 */
PBS::PBS(std::shared_ptr<Instance> user_instance_ptr,
         int single_agent_solver_name, double cutoff_time,
         shared_ptr<RobotMotion> bot_motion, int screen)
    : screen(screen), bot_motion(bot_motion) {
    instance_ptr = user_instance_ptr;
    if (single_agent_solver_name == 0) {
        single_agent_solver = "BAS";
    } else if (single_agent_solver_name == 1) {
        single_agent_solver = "BCS";
    } else {
        std::cerr << "Invalid single agent solver!\n";
        exit(-1);
    }
    sipp_ptr =
        std::make_shared<SIPP>(user_instance_ptr, cutoff_time / 10, bot_motion);
    num_of_agents = (int)instance_ptr->agents.size();
    cutoff_runtime = cutoff_time;
    this->time_limit = cutoff_time;
    need_replan.resize(num_of_agents);
    std::fill(need_replan.begin(), need_replan.end(), true);
    agents_arrived.resize(num_of_agents);
    std::fill(agents_arrived.begin(), agents_arrived.end(), false);
}

void PBS::clear() {
    priority_graph.clear();
    ordered_agents.clear();
    need_replan.clear();
    agents_arrived.clear();
    sipp_ptr->Reset();
    sipp_ptr = nullptr;
    instance_ptr = nullptr;
    runtime = 0;
    runtime_generate_child = 0;
    runtime_build_CT = 0;
    runtime_build_CAT = 0;
    runtime_path_finding = 0;
    runtime_detect_conflicts = 0;
    runtime_preprocessing = 0;
    num_HL_expanded = 0;
}

bool PBS::SolveSingleAgent(PTNode& node, std::set<int>& rtp, int agent_id,
                           bool log) {
    assert(need_replan[agent_id]);
    ReservationTable rt(instance_ptr->graph->map_size);
    node.getRTFromP(rt, rtp, instance_ptr->simulation_window);
    // Not needed?
    // InsertInitLocation(agent_id, *instance_ptr, rt);
    if (log)
        printRT(rt);
    double solution_cost = 0.0;
    Path path;
    bool sipp_success =
        sipp_ptr->run(agent_id, rt, node.motion_solution[agent_id], path,
                      solution_cost, node.all_agents_timed_path[agent_id], log);
    if (!sipp_success) {
        return false;
    }

    node.plan[agent_id].clear();
    node.plan[agent_id] = path;
    node.solution_cost[agent_id] = solution_cost;

    ReservationTable rtdebug(instance_ptr->graph->map_size);
    node.getRTFromP(rtdebug, rtp, instance_ptr->simulation_window);
    if (!checkValid(rtdebug, path, agent_id)) {
        std::fstream outputRT("ReservationTable.txt",
                              std::fstream::in | std::fstream::out |
                                  std::fstream::trunc | std::fstream::binary);
        for (auto itt = rt.begin(); itt != rt.end(); ++itt) {
            for (auto itt2 = itt->begin(); itt2 != itt->end(); ++itt2) {
                // std::cout<<"writting!\n";
                outputRT.write((char*)&itt2->t_min, sizeof(itt2->t_min));
                outputRT << ',';
                outputRT.write((char*)&itt2->t_max, sizeof(itt2->t_max));
                outputRT << ',' << itt2->agent_id << ';';
            }
            outputRT << '\n';
        }

        outputRT.close();
        std::cout << "replanned agent " << agent_id << '\n';
        std::cout << "rtp: ";
        for (auto itt = rtp.begin(); itt != rtp.end(); ++itt) {
            std::cout << *itt << " ";
        }
        printValidRT(rt);
        std::cout << "agent " << agent_id << '\n';
        printPath(path);
        exit(-1);
    }
    return true;
}

/**
 * Update the path that is planned at each node
 *
 * @param node The node of the priority tree that needs to be modified
 * @param index The index of the agent
 * @return If success return True
 */
bool PBS::UpdatePlan(PTNode& node, int index) {
    std::list<int> priority_list = node.topologicalSort(index);
    ReservationTable tmp_rt(instance_ptr->graph->map_size);
    std::set<int> higher_agents;
    for (int agent_id : priority_list) {
        if (agent_id == index or
            node.checkValid(tmp_rt, agent_id,
                            instance_ptr->simulation_window) != -1) {
            bool success = SolveSingleAgent(node, higher_agents, agent_id);
            if (not success) {
                return false;
            }
        }
        higher_agents.insert(agent_id);
        node.insertPathToRT(tmp_rt, agent_id, instance_ptr->simulation_window);
    }
    return true;
}

bool PBS::initRootNode(std::shared_ptr<PTNode>& root_node) {
    vector<Path> plan(num_of_agents);
    // int: the id of agent, std::set<int>: the agents with higher priority
    std::map<int, std::set<int>> priority;
    for (int i = 0; i < num_of_agents; i++) {
        priority[i] = std::set<int>();
    }

    std::vector<MotionInfo> init_bezier(num_of_agents);
    std::vector<TimedPath> init_timed_path(num_of_agents);

    root_node =
        std::make_shared<PTNode>(plan, init_bezier, init_timed_path, priority);
    for (int i = 0; i < num_of_agents; ++i) {
        if (not need_replan[i]) {
            continue;
        }
        if (!UpdatePlan(*root_node, i)) {
            spdlog::error("Fail to find a initial plan for agent {}!", i);
            // set<int> high_agts;
            // SolveSingleAgent(*root_node, high_agts, i, true);
            return false;
        }
    }
    root_node->calculateCost(instance_ptr);
    root_cost = root_node->cost;
    root_node->parent = nullptr;
    for (int a1 = 0; a1 < num_of_agents; a1++) {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++) {
            if (root_node->CheckCollision(a1, a2)) {
                root_node->conflicts.emplace_back(new Conflict(a1, a2));
            }
        }
    }
    root_cost = std::accumulate(root_node->solution_cost.begin(),
                                root_node->solution_cost.end(), 0.0);
    return true;
}

void PBS::InsertInitLocation(int agent_id, Instance& instance,
                             ReservationTable& rt) {
    for (Agent& tmp_agent : instance.agents) {
        for (auto entry : tmp_agent.previous_path) {
            TimeInterval tmp_block;
            tmp_block.t_max = entry.leaving_time_tail;
            tmp_block.t_min = entry.arrival_time;
            tmp_block.agent_id = tmp_agent.id;
            rt[entry.location].push_back(tmp_block);
        }

        if (agent_id == tmp_agent.id or agents_arrived[tmp_agent.id]) {
            continue;
        }
        // avoid take the initial position collision
        TimeInterval newTI;
        newTI.t_max = tmp_agent.earliest_start_time + 2.8284;
        newTI.t_min = tmp_agent.earliest_start_time;
        newTI.agent_id = tmp_agent.id;
        rt[tmp_agent.start_location].push_back(newTI);
    }
}

inline bool PBS::isTerminate(std::shared_ptr<PTNode> curr_n) {
    if (curr_n->conflicts.empty()) {
        solution_found = true;

        if (!curr_n->checkSolution(*instance_ptr)) {
            spdlog::error("Solution is not valid!");
            exit(-1);
        }
        return true;
    } else {
        return false;
    }
}

std::shared_ptr<PTNode> PBS::selectNode() {
    std::shared_ptr<PTNode> curr = open_list.top();
    open_list.pop();
    priority_graph.assign(num_of_agents, vector<bool>(num_of_agents, false));
    for (auto tmp_n = curr; tmp_n != nullptr; tmp_n = tmp_n->parent) {
        if (tmp_n->parent != nullptr)  // non-root node
            priority_graph[tmp_n->constraint.low][tmp_n->constraint.high] =
                true;
    }
    num_HL_expanded++;
    return curr;
}

shared_ptr<Conflict> PBS::chooseConflict(const PTNode& node) {
    if (node.conflicts.empty())
        return nullptr;
    return node.conflicts.back();
}

inline void PBS::pushNode(const std::shared_ptr<PTNode>& node) {
    // update handles
    open_list.push(node);
}

void PBS::pushNodes(const std::shared_ptr<PTNode>& n1,
                    const std::shared_ptr<PTNode>& n2) {
    if (n1 != nullptr and n2 != nullptr) {
        if (n1->cost < n2->cost) {
            pushNode(n2);
            pushNode(n1);
        } else {
            pushNode(n1);
            pushNode(n2);
        }
    } else if (n1 != nullptr) {
        pushNode(n1);
    } else if (n2 != nullptr) {
        pushNode(n2);
    }
}

void PBS::topologicalSort(list<int>& stack) {
    stack.clear();
    vector<bool> visited(num_of_agents, false);

    // Call the recursive helper function to store Topological
    // Sort starting from all vertices one by one
    for (int i = 0; i < num_of_agents; i++) {
        if (!visited[i])
            topologicalSortUtil(i, visited, stack);
    }
}

void PBS::topologicalSortUtil(int v, vector<bool>& visited, list<int>& stack) {
    // Mark the current node as visited.
    visited[v] = true;

    // Recur for all the vertices adjacent to this vertex
    assert(!priority_graph.empty());
    for (int i = 0; i < num_of_agents; i++) {
        if (priority_graph[v][i] and !visited[i])
            topologicalSortUtil(i, visited, stack);
    }
    // Push current vertex to stack which stores result
    stack.push_back(v);
}

void PBS::getHigherPriorityAgents(const list<int>::reverse_iterator& p1,
                                  set<int>& higher_agents) {
    for (auto p2 = std::next(p1); p2 != ordered_agents.rend(); ++p2) {
        if (priority_graph[*p1][*p2]) {
            auto ret = higher_agents.insert(*p2);
            if (ret.second)  // insert successfully
            {
                getHigherPriorityAgents(p2, higher_agents);
            }
        }
    }
}

void PBS::getLowerPriorityAgents(const list<int>::iterator& p1,
                                 set<int>& lower_subplans) {
    for (auto p2 = std::next(p1); p2 != ordered_agents.end(); ++p2) {
        if (priority_graph[*p2][*p1]) {
            auto ret = lower_subplans.insert(*p2);
            if (ret.second)  // insert successfully
            {
                getLowerPriorityAgents(p2, lower_subplans);
            }
        }
    }
}

/**
 * @brief return true if agent low is lower than agent high
 *
 * */
bool PBS::hasHigherPriority(int low, int high) const {
    std::queue<int> Q;
    vector<bool> visited(num_of_agents, false);
    visited[low] = false;
    Q.push(low);
    while (!Q.empty()) {
        auto n = Q.front();
        Q.pop();
        if (n == high)
            return true;
        for (int i = 0; i < num_of_agents; i++) {
            if (priority_graph[n][i] and !visited[i])
                Q.push(i);
        }
    }
    return false;
}

bool PBS::generateChild(int child_id, std::shared_ptr<PTNode> parent, int low,
                        int high) {
    assert(child_id == 0 or child_id == 1);
    std::shared_ptr<PTNode> node = std::make_shared<PTNode>(parent);
    node->parent = parent;
    node->curr_conflict = std::make_shared<Conflict>(low, high);
    if (child_id == 0)
        parent->children.first = node;
    else
        parent->children.second = node;

    node->constraint.set(low, high);
    priority_graph[high][low] = false;
    priority_graph[low][high] = true;
    if (screen > 2)
        printPriorityGraph();
    topologicalSort(ordered_agents);
    if (screen > 2) {
        cout << "Ordered agents: ";
        for (int i : ordered_agents)
            cout << i << ",";
        cout << endl;
    }
    vector<int> topological_orders(
        num_of_agents);  // map agent i to its position in ordered_agents
    auto i = num_of_agents - 1;
    for (const auto& a : ordered_agents) {
        topological_orders[a] = i;
        i--;
    }

    std::priority_queue<pair<int, int>>
        to_replan;  // <position in ordered_agents, agent id>
    vector<bool> lookup_table(num_of_agents, false);
    to_replan.emplace(topological_orders[low], low);
    lookup_table[low] = true;
    {  // find conflicts where one agent is higher than high and the other agent
       // is lower than low
        set<int> higher_agents;
        auto p = ordered_agents.rbegin();
        std::advance(p, topological_orders[high]);
        assert(*p == high);
        getHigherPriorityAgents(p, higher_agents);
        higher_agents.insert(high);

        set<int> lower_agents;
        auto p2 = ordered_agents.begin();
        std::advance(p2, num_of_agents - 1 - topological_orders[low]);
        assert(*p2 == low);
        getLowerPriorityAgents(p2, lower_agents);

        for (const auto& conflict : node->conflicts) {
            int a1 = conflict->a1;
            int a2 = conflict->a2;
            if (a1 == low or a2 == low)
                continue;
            if (topological_orders[a1] > topological_orders[a2]) {
                std::swap(a1, a2);
            }
            if (!lookup_table[a1] and
                lower_agents.find(a1) != lower_agents.end() and
                higher_agents.find(a2) != higher_agents.end()) {
                to_replan.emplace(topological_orders[a1], a1);
                lookup_table[a1] = true;
            }
        }
    }

    while (!to_replan.empty()) {
        int a, rank;
        tie(rank, a) = to_replan.top();
        to_replan.pop();
        lookup_table[a] = false;
        if (screen > 2)
            cout << "Replan agent " << a << endl;
        // Re-plan path
        set<int> higher_agents;
        auto p = ordered_agents.rbegin();
        std::advance(p, rank);
        assert(*p == a);
        getHigherPriorityAgents(p, higher_agents);
        assert(!higher_agents.empty());
        if (screen > 2) {
            cout << "Higher agents: ";
            for (auto i : higher_agents)
                cout << i << ",";
            cout << endl;
        }
        if (!SolveSingleAgent(*node, higher_agents, a)) {
            // Debug: run again with logging turned on
            if (screen > 2) {
                spdlog::warn("Fail to find a path for agent {}! Replanning "
                             "with logging.",
                             a);
                SolveSingleAgent(*node, higher_agents, a, true);
            }

            if (child_id == 0)
                parent->children.first = nullptr;
            else
                parent->children.second = nullptr;
            return false;
        }

        // Delete old conflicts
        for (auto c = node->conflicts.begin(); c != node->conflicts.end();) {
            if ((*c)->a1 == a or (*c)->a2 == a)
                c = node->conflicts.erase(c);
            else
                ++c;
        }

        // Update conflicts and to_replan
        set<int> lower_agents;
        auto p2 = ordered_agents.begin();
        std::advance(p2, num_of_agents - 1 - rank);
        assert(*p2 == a);
        getLowerPriorityAgents(p2, lower_agents);
        if (screen > 2 and !lower_agents.empty()) {
            cout << "Lower agents: ";
            for (auto i : lower_agents)
                cout << i << ",";
            cout << endl;
        }

        // Find new conflicts
        for (auto a2 = 0; a2 < num_of_agents; a2++) {
            if (a2 == a or lookup_table[a2] or
                higher_agents.count(a2) >
                    0)  // already in to_replan or has higher priority
                continue;
            auto t = clock();
            if (node->CheckCollision(a, a2)) {
                node->conflicts.emplace_back(new Conflict(a, a2));
                if (lower_agents.count(a2) >
                    0)  // has a collision with a lower priority agent
                {
                    // if (screen > 1)
                    //     cout << "\t" << a2
                    //          << " needs to be replanned due to collisions
                    //          with "
                    //          << a << endl;
                    to_replan.emplace(topological_orders[a2], a2);
                    lookup_table[a2] = true;
                }
            }
            runtime_detect_conflicts += (double)(clock() - t) / CLOCKS_PER_SEC;
        }
    }
    num_HL_generated++;
    return true;
}

void PBS::printPriorityGraph() const {
    cout << "Priority graph:";
    for (int a1 = 0; a1 < num_of_agents; a1++) {
        for (int a2 = 0; a2 < num_of_agents; a2++) {
            if (priority_graph[a1][a2])
                cout << a1 << "<" << a2 << ",";
        }
    }
    cout << endl;
}

void PBS::recursivePrint(const std::shared_ptr<PTNode>& curr_node) {
    std::shared_ptr<PTNode> tmp_node = curr_node;
    cout << "[INFO] ";
    while (tmp_node->parent != nullptr) {
        cout << "Conflict: " << *(tmp_node->curr_conflict) << "; ";
        tmp_node = tmp_node->parent;
    }
    cout << endl;
}

bool PBS::solve(const string& outputFileName) {
    // if (screen > 0)  // 1 or 2
    // {
    //     string name = getSolverName();
    //     name.resize(35, ' ');
    //     cout << name << ": ";
    // }
    // set timer
    // Print motion model
    if (screen > 1) {
        spdlog::info("Motion model: V_MIN: {}, V_MAX: {}, A_MAX: {}, "
                     "ROTATE_COST: {}, TURN_BACK_COST: {}",
                     bot_motion->V_MIN, bot_motion->V_MAX, bot_motion->A_MAX,
                     bot_motion->ROTATE_COST, bot_motion->TURN_BACK_COST);
    }
    auto start = clock();

    std::shared_ptr<PTNode> Root;
    if (not initRootNode(Root)) {
        // printf("[Error] Fail to find a initial plan!\n");
        spdlog::error("Fail to find a initial plan!");
        // exit(-1);
        return false;
    }
    if (screen > 2)
        spdlog::info("Root generated with cost: {}", root_cost);
    open_list.push(Root);
    while (!open_list.empty() and ((double)(clock() - start) / CLOCKS_PER_SEC) <
                                      this->cutoff_runtime) {
        auto curr = selectNode();
        if (this->isTerminate(curr)) {
            solution_node = curr;
            break;
        }

        curr->conflict = chooseConflict(*curr);

        // if (screen > 0) {
        //     recursivePrint(curr);
        //     cout << "	Expand " << curr->depth << "	on "
        //          << *(curr->conflict) << endl;
        // }
        auto t1 = clock();
        generateChild(0, curr, curr->conflict->a1, curr->conflict->a2);
        generateChild(1, curr, curr->conflict->a2, curr->conflict->a1);
        runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
        pushNodes(curr->children.first, curr->children.second);
    }  // end of while loop
    runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
    return solution_found;
}

void PBS::updateCost() {
    solution_cost = 0;
    if (solution_node != nullptr) {
        for (auto& tmp_cost : solution_node->solution_cost) {
            solution_cost += tmp_cost;
        }
    }
    num_LL_expanded = sipp_ptr->count_node_expanded;
    num_LL_re_expand = sipp_ptr->count_node_re_expand;
    num_LL_generated = sipp_ptr->count_node_generated;
}

void PBS::saveTimedPath(const string& file_name) const {
    std::ofstream output;
    output.open(file_name);
    solution_node->plan;
    for (int i = 0; i < solution_node->all_agents_timed_path.size(); i++) {
        output << "Agent " << i << ":";
        for (const auto& state : solution_node->all_agents_timed_path[i])
            output << "(" << instance_ptr->graph->getRowCoordinate(state.first)
                   << "," << instance_ptr->graph->getColCoordinate(state.first)
                   << "," << state.second << ")->";
        output << endl;
    }
    output.close();
}

vector<vector<tuple<int, int, double, int>>> PBS::getTimedPath() const {
    if (screen > 0) {
        spdlog::info("############## MASS: Plan result ##############");
    }

    // Print raw path (without window)
    if (screen > 0) {
        cout << "Raw path (without simulation window):" << endl;
        for (int i = 0; i < solution_node->plan.size(); i++) {
            cout << "Agent " << i << ": ";
            for (const auto& state : solution_node->plan[i]) {
                cout << "("
                     << instance_ptr->graph->getRowCoordinate(state.location)
                     << ","
                     << instance_ptr->graph->getColCoordinate(state.location)
                     << "," << state.arrival_time << ","
                     << state.leaving_time_tail << ")->";
            }
            cout << endl;
        }
    }

    vector<vector<tuple<int, int, double, int>>> timed_path;
    for (int i = 0; i < solution_node->all_agents_timed_path.size(); i++) {
        // output << "Agent " << i << ":";
        vector<tuple<int, int, double, int>> agent_path;
        int goal_id = 0;
        // for (const auto& state : solution_node->plan[i]) {
        for (int j = 0; j < solution_node->plan[i].size(); j++) {
            auto state = solution_node->plan[i][j];
            if (state.arrival_time >= instance_ptr->simulation_window) {
                // If the agent arrives after the simulation window, we do not
                // record it
                continue;
            }
            int loc = state.location;
            double t;
            // Not the last state, take the next arrival time minus the max
            // time it takes to move as the actual start time of the action
            // from current state to the next state
            if (j < solution_node->plan[i].size() - 1)
                t = solution_node->plan[i][j + 1].arrival_time -
                    (1 / bot_motion->V_MAX);
            // Last state, take the arrival time of the current state as the
            // start time of the action
            else
                t = state.arrival_time;
            int task_id = -1;
            // Reached a goal
            auto curr_goals = instance_ptr->agents[i].goal_locations;
            if (curr_goals.size() > goal_id and
                curr_goals[goal_id].loc == loc) {
                task_id = curr_goals[goal_id].id;
                goal_id++;
            }
            agent_path.push_back(make_tuple(
                instance_ptr->graph->getRowCoordinate(loc),
                instance_ptr->graph->getColCoordinate(loc), t, task_id));
        }

        if (agent_path.empty()) {
            // If the agent has no path, we still need to add the initial state
            // with a dummy time and task ID
            agent_path.push_back(
                make_tuple(instance_ptr->graph->getRowCoordinate(
                               instance_ptr->agents[i].start_location),
                           instance_ptr->graph->getColCoordinate(
                               instance_ptr->agents[i].start_location),
                           0.0, -1));
        }

        timed_path.push_back(agent_path);
    }

    // Print the final timed path
    if (screen > 0) {
        cout << "Final timed path:" << endl;
        for (int i = 0; i < timed_path.size(); i++) {
            cout << "Agent " << i << ": ";
            for (const auto& state : timed_path[i]) {
                cout << "(" << get<0>(state) << "," << get<1>(state) << ","
                     << get<2>(state) << ")->";
                if (get<3>(state) >= 0) {
                    cout << "End of task " << get<3>(state) << " at time "
                         << get<2>(state) << " ";
                }
            }
            cout << endl;
        }
    }

    return timed_path;
}

void PBS::savePath(const string& file_name) const {
    std::ofstream output;
    output.open(file_name);
    solution_node->plan;
    for (int i = 0; i < solution_node->plan.size(); i++) {
        output << "Agent " << i << ":";
        for (const auto& state : solution_node->plan[i])
            output << "("
                   << instance_ptr->graph->getRowCoordinate(state.location)
                   << ","
                   << instance_ptr->graph->getColCoordinate(state.location)
                   << "," << state.arrival_time << ","
                   << state.leaving_time_tail << ")->";
        output << endl;
    }
    output.close();
}

void PBS::saveResults(int all_agents_solved, const string& fileName,
                      const string& instanceName) const {
    std::ifstream infile(fileName);
    bool exist = infile.good();
    infile.close();
    if (!exist) {
        ofstream addHeads(fileName);
        addHeads << "#agents, runtime,is solved,"
                    "#high-level expanded,#high-level generated,"
                    "#low-level expanded,#low-level generated,"
                    "#low-level re-expanded,#low-level motion solver called,"
                 << "solution cost,root g value,#PBS-called,"
                 << "runtime of detecting conflicts,runtime of building "
                    "constraint tables,runtime of building CATs,"
                 << "runtime of path finding,runtime of generating child nodes,"
                 << "preprocessing runtime,"
                    "#low-level expand runtime,#low-level motion runtime,"
                    "solver name,instance name"
                 << endl;
        addHeads.close();
    }
    ofstream stats(fileName, std::ios::app);
    stats << instance_ptr->num_of_agents << "," << runtime << ","
          << all_agents_solved << ", " << num_HL_expanded << ","
          << num_HL_generated << "," << num_LL_expanded << ","
          << num_LL_generated << "," << sipp_ptr->count_node_re_expand << ","
          << sipp_ptr->count_motion_solver << "," << solution_cost << ","
          << root_cost << "," << num_PBS_called << ","
          << runtime_detect_conflicts << "," << runtime_build_CT << ","
          << runtime_build_CAT << "," << sipp_ptr->total_runtime_ << ","
          << runtime_generate_child << "," << runtime_preprocessing << ","
          << sipp_ptr->expand_runtime << "," << sipp_ptr->motion_solver_runtime
          << "," << getSolverName() << "," << instanceName << endl;
    stats.close();
}

void PBS::printPath(Path path) {
    std::cout << "\n\n_________________________________printing "
                 "Path____________________________________\n";
    std::cout << "path size: " << path.size() << "\n";
    for (auto itt = path.begin(); itt != path.end(); ++itt) {
        std::cout << "cp" << itt->location << "\tarrival: " << itt->arrival_time
                  << "\tleave: " << itt->leaving_time_tail << "\n";
    }
    std::cout << "_____________________________________________________________"
                 "_____________________\n\n";
}

void PBS::printRT(ReservationTable rt) {
    std::cout << "\n\n_________________________________printing "
                 "ReservationTable PBS_________________________\n";
    for (int i = 0; i < (signed)rt.size(); ++i) {
        std::cout << "loc (" << instance_ptr->graph->getRowCoordinate(i) << ", "
                  << instance_ptr->graph->getColCoordinate(i) << ")\t";
        for (auto ittemp = rt[i].begin(); ittemp != rt[i].end(); ++ittemp) {
            std::cout << "t_min=" << ittemp->t_min << "\t"
                      << "t_max=" << ittemp->t_max << "\t#"
                      << "agent id = " << ittemp->agent_id << "#\t";
        }
        std::cout << "\n";
    }
    std::cout << "_____________________________________________________________"
                 "________________________\n\n";
}

void PBS::printValidRT(ReservationTable rt) {
    std::cout << "\n\n_________________________________printing "
                 "ReservationTable_________________________\n";
    for (int i = 0; i < (signed)rt.size(); ++i) {
        if (rt[i].empty()) {
            continue;
        }
        std::cout << "cp" << i << "\t";
        for (auto ittemp = rt[i].begin(); ittemp != rt[i].end(); ++ittemp) {
            std::cout << ittemp->t_min << "\t" << ittemp->t_max << "\t#"
                      << ittemp->agent_id << "#\t";
        }
        std::cout << "\n";
    }
    std::cout << "_____________________________________________________________"
                 "________________________\n\n";
}

/**
 * Print the order of the vehicles at each start point
 */
void PBS::printPriority(std::map<int, std::set<int>> p) {
    std::cout << "\n\n_________________________________printing "
                 "Priority_______________________________\n";
    for (auto it = p.begin(); it != p.end(); ++it) {
        std::cout << it->first << ": ";
        for (auto it2 = it->second.begin(); it2 != it->second.end(); ++it2)
            std::cout << *it2 << " ";
        std::cout << "\n";
    }
    std::cout << "_____________________________________________________________"
                 "____________________\n\n";
}

bool PBS::checkValid(ReservationTable& rt, Path& path, int agent) {
    for (auto& path_entry : path) {
        for (auto& rt_interval : rt[path_entry.location]) {
            assert(rt_interval.agent_id != agent);
            if (path_entry.leaving_time_tail - rt_interval.t_min >= EPSILON and
                rt_interval.t_max - path_entry.arrival_time >= EPSILON) {
                std::cout << "agent " << agent << ": "
                          << " loc = ("
                          << instance_ptr->graph->getRowCoordinate(
                                 path_entry.location)
                          << ", "
                          << instance_ptr->graph->getColCoordinate(
                                 path_entry.location)
                          << "), arrival = " << path_entry.arrival_time << ", "
                          << ", leave = " << path_entry.leaving_time_tail
                          << '\n'
                          << "agent " << rt_interval.agent_id << ": "
                          << rt_interval.t_min << " " << rt_interval.t_max
                          << '\n';
                return false;
            }
        }
    }
    return true;
}
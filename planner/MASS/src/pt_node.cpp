#include "pt_node.h"

PTNode::PTNode(const vector<Path>& pl,
				std::vector<MotionInfo>& solution,
                std::vector<TimedPath>& timed_paths,
				const std::map<int, std::set<int>>& pr){
	plan = pl;
	priority = pr;
    motion_solution = solution;
    all_agents_timed_path = timed_paths;
    solution_cost.resize(solution.size(), 0);
}

PTNode::~PTNode()
{
    // Clear the conflicts list
    for (auto& conflict : conflicts) {
        conflict.reset();
    }
    conflicts.clear();
    // Clear the children nodes
    children.first.reset();
    children.second.reset();
    parent.reset();
    motion_solution.clear();
}


// /***
//  * Finish this after working the structure
// */
// void PTNode::writeToFile(Instance& instance, const string& file_name)
// {
// 	std::ifstream infile(file_name);
// 	bool exist = infile.good();
// 	infile.close();
// 	if (!exist) {
// 		ofstream addHeads(file_name);
// 		addHeads << "Num of agents: " << instance.agents.size() << std::endl;
// 		addHeads << "Agent id; Start Point;Goal Point;Length; Start time; Control Points; Trajectory" << '\n';
// 		addHeads.close();
// 	}

// 	ofstream outfile(file_name, std::ios::app);
// 	for(int i = 0; i < (signed)plan.size(); ++i){
// 		outfile <<  i \
// 			<< ";" << instance.agents[i].start_location  \
// 			<< ";" << instance.agents[i].goal_location  \
// 			<< ";" << instance.agents[i].length \
// 			<< ";" << instance.agents[i].earliest_start_time << ";";
// 		outfile << "; ";
// 		for (auto tmp_node : plan[i]) {
// 			outfile << tmp_node.location << ", ";
// 		}
// 		outfile << '\n';
// 	}
// 	outfile.close();	
// }

/***
 * Generate the priority list for all the agents
 * 
 * @param v The id of agent need to be sort
 * @param visited The list to indicate visited agents
 * @param List The already sorted list
*/
void PTNode::topologicalSortUtil(int v, std::vector<bool>& visited, std::list<int>& List)
{
    // Mark the current node as visited.
    visited[v] = true;
 
    // Recur for all the vertices
    // adjacent to this vertex
    for (auto agent_id : priority[v]) {
		if (!visited[agent_id]) {
            topologicalSortUtil(agent_id, visited, List);
		}
	}

    // Push current vertex to stack
    // which stores result priority
    List.push_back(v);
}

bool PTNode::CheckCollision(int agent_first, int agent_second)
{
    for(auto first_agent_node = plan[agent_first].begin(); first_agent_node != plan[agent_first].end(); ++first_agent_node){
        for (auto second_agent_node = plan[agent_second].begin(); second_agent_node != plan[agent_second].end(); ++second_agent_node) {
            if( first_agent_node->location == second_agent_node->location){
                if(first_agent_node->leaving_time_tail - second_agent_node->arrival_time >= EPSILON &&
                   second_agent_node->leaving_time_tail - first_agent_node->arrival_time >= EPSILON) {
                    return true;
                }
            }
        }
    }
    return false;
}

void PTNode::printConflicts()
{
    for (const auto& tmp_conflict : this->conflicts)
    {
        cout << *tmp_conflict << endl;
    }
}

/**
 * @brief Get the priority order
 *
 * @param agent
 * @return
 */
std::list<int> PTNode::topologicalSort(int agent){
	std::list<int> List;
 
    // Mark all the vertices as not visited
    std::vector<bool> visited (plan.size(), false);

    // Call the recursive helper function
    // to store Topological
    // Sort starting from all
    // vertices one by one
    topologicalSortUtil(agent, visited, List);
    return List;
}

/**
 * Calculate the time cost of the current plan
 * 
*/
void PTNode::calculateCost(){
	double c = 0;
	for(auto it = plan.begin(); it != plan.end(); ++it){
		if (!it->empty()){
			c += it->back().leaving_time_tail;
		}
	}
	cost = c;
}

void PTNode::calculateCost(std::shared_ptr<Instance> instance_ptr){
	double c = 0;
	for(unsigned int i = 0; i < instance_ptr->agents.size(); i++){
		if (!instance_ptr->agents[i].is_solved) {
			c += instance_ptr->agents[i].tmp_cost;
		}
	}
	cost = c;
}


void printRT(ReservationTable rt){
	std::cout<<"\n\n_________________________________printing ReservationTable_________________________\n";
	for(int i = 0; i < (signed) rt.size(); ++i){
		std::cout<<"cp" << i << "\t";
		for(auto ittemp = rt[i].begin(); ittemp != rt[i].end(); ++ittemp){
			std::cout << "t_min=" << ittemp->t_min << "\t" << "tmax=" <<ittemp->t_max << "\t" << "agent id=" << ittemp->agent_id << "\t";
		}
		std::cout<<"\n";
	}
	std::cout<<"_____________________________________________________________________________________\n\n";
}

void PTNode::printPlan(){
	std::cout<<"\n\n_________________________________printing Plans_________________________\n";
	for(int i = 0; i < (signed) plan.size(); ++i){
		std::cout<<"Agent " << i << ": \t";
		for(PathEntry tmp_path : plan[i]){
			printf("Loc: %d, From %f to %f\t", tmp_path.location, tmp_path.arrival_time, tmp_path.leaving_time_tail);
		}
		std::cout<<"\n";
	}
	std::cout<<"_____________________________________________________________________________________\n\n";
}

int PTNode::checkValid(ReservationTable& rt, int agent_id,
                       int simulation_window) {
    for (auto& path_entry: plan[agent_id]) {
        for (auto& rt_interval: rt[path_entry.location]) {
            if (rt_interval.agent_id == agent_id) continue;
            if (simulation_window > 0 &&
                path_entry.arrival_time >= simulation_window)
                continue;
            if(path_entry.leaving_time_tail - rt_interval.t_min >= EPSILON and
               rt_interval.t_max - path_entry.arrival_time >= EPSILON) {
                spdlog::error("Agent {} at loc {}: {} - {}, Agent {}: {} - {}",
                          agent_id, path_entry.location, path_entry.arrival_time, path_entry.leaving_time_tail,
                          rt_interval.agent_id, rt_interval.t_min, rt_interval.t_max);
                return rt_interval.agent_id;
            }
        }
    }
    return -1;
}

bool PTNode::checkSolution(Instance& instance){
    std::pair<int, int> result(-1, -1);
    ReservationTable rt_table(instance.graph->map_size);
    std::set<int> s;
    for(int i = 0; i < (signed) plan.size();++i) s.insert(i);
    getRTFromP(rt_table, s, instance.simulation_window);
    for (size_t agent_id = 0; agent_id < plan.size(); agent_id++) {
        int conflict_id = checkValid(rt_table, agent_id,
                                     instance.simulation_window);
        if (conflict_id != -1) {
            return false;
        }
    }
    // printRT(rt_table);
    return true;
}

std::pair<int, int> PTNode::getFirstCollision(Instance& instance){
    std::pair<int, int> result(-1, -1);
	ReservationTable rt_table(instance.graph->map_size);
	std::set<int> s;
	for(int i = 0; i < (signed) plan.size();++i) s.insert(i);
	getRTFromP(rt_table, s, instance.simulation_window);
    for (size_t agent_id = 0; agent_id < plan.size(); agent_id++) {
        int conflict_id = checkValid(rt_table, agent_id,
                                     instance.simulation_window);
        if (conflict_id != -1) {
            result = std::pair<int, int>(agent_id, conflict_id);
            return result;
        }
    }
	return result;
}

void PTNode::printPriorityMap()
{
	for (auto tmp_priority: priority) {
		printf("agent %d: ", tmp_priority.first);
		for (auto item : tmp_priority.second) {
			printf("agent %d\t", item);
		}
		std::cout << std::endl;
	}
}

/**
 * @briefSet Retrieve the set of index with higher priority
 * 
 * @param index The index of the current agent
 * @param[out] p The set of index with higher priority
*/
void PTNode::getRTP(std::set<int> &p, int index){
	for (int i = 0; i < (signed) plan.size(); ++i){
		//not already in the list
		if(p.find(i) == p.end() && (priority[i].find(index) != priority[i].end())){
			p.insert(i);
			getRTP(p, i);
		}
	}
}

/**
 * @brief Get the reservation table given the agents priority
 *
 * @param p The agent with higher priority
 * @param[out] rt The reservation table
*/
void PTNode::getRTFromP(ReservationTable& rt, std::set<int>& p,
                        int simulation_window){
	for(int agent_id: p){
        insertPathToRT(rt, agent_id, simulation_window);
	}
}

void PTNode::insertPathToRT(ReservationTable& rt, int agent_id,
                            int simulation_window)
{
    for(auto it2 = plan[agent_id].begin(); it2 != plan[agent_id].end(); ++it2){
        TimeInterval newTI;

        newTI.t_max = it2->leaving_time_tail;
        newTI.t_min = it2->arrival_time;
        if (simulation_window <= 0 || newTI.t_min < simulation_window) {
            newTI.agent_id = agent_id;
            rt[it2->location].push_back(newTI);
        }
    }
}
#include "SingleAgentSolver.h"

list<int> SingleAgentSolver::getNextLocations(
    int curr) const  // including itself and its neighbors
{
    list<int> rst = instance.graph->getNeighbors(curr);
    rst.emplace_back(curr);
    return rst;
}

// void SingleAgentSolver::compute_heuristics()
// {
// 	struct Node
// 	{
// 		int location;
// 		double value;

// 		Node() = default;
// 		Node(int location, double value) : location(location),
// value(value) {}
// 		// the following is used to comapre nodes in the OPEN list
// 		struct compare_node
// 		{
// 			// returns true if n1 > n2 (note -- this gives us
// *min*-heap). 			bool operator()(const Node& n1, const
// Node& n2) const
// 			{
// 				return n1.value >= n2.value;
// 			}
// 		};  // used by OPEN (heap) to compare nodes (top of the heap has
// min f-val, and then highest g-val)
// 	};

// 	my_heuristic.resize(instance.map_size, MAX_TIMESTEP);

// 	// generate a heap that can save nodes (and a open_handle)
// 	boost::heap::pairing_heap< Node,
// boost::heap::compare<Node::compare_node> > heap;

// 	Node root(goal_location, 0);
// 	my_heuristic[goal_location] = 0;
// 	heap.push(root);  // add root to heap
// 	while (!heap.empty())
// 	{
// 		Node curr = heap.top();
// 		heap.pop();
// 		for (int next_location : instance.getNeighbors(curr.location))
// 		{
// 			if (my_heuristic[next_location] > curr.value + 1)
// 			{
// 				my_heuristic[next_location] = curr.value + 1;
// 				Node next(next_location, curr.value + 1);
// 				heap.push(next);
// 			}
// 		}
// 	}
// }

// void SingleAgentSolver::compute_heuristics() {
//     struct Node {
//         int location;
//         double value;

//         Node() = default;
//         Node(int location, double value) : location(location), value(value) {
//         }
//         // the following is used to comapre nodes in the OPEN list
//         struct compare_node {
//             // returns true if n1 > n2 (note -- this gives us *min*-heap).
//             bool operator()(const Node* n1, const Node* n2) const {
//                 return n1->value >= n2->value;
//             }
//         };  // used by OPEN (heap) to compare nodes (top of the heap has min
//             // f-val, and then highest g-val)

//         struct EqNode {
//             bool operator()(const Node* n1, const Node* n2) const {
//                 return (n1 == n2) || (n1 && n2 && n1->location ==
//                 n2->location);
//             }
//         };

//         // The following is used to generate the hash value of a node
//         struct Hasher {
//             std::size_t operator()(const Node* n) const {
//                 return std::hash<int>()(n->location);
//             }
//         };

//         fibonacci_heap<Node*, compare<Node::compare_node> >::handle_type
//             open_handle;
//     };

//     my_heuristic.resize(instance.graph.map_size, DBL_MAX);

//     // std::cout << "start computing h for loc = "<< goal_location
//     <<std::endl; fibonacci_heap<Node*, compare<Node::compare_node> > heap;
//     unordered_set<Node*, Node::Hasher, Node::EqNode> nodes;

//     // if(consider_rotation)
//     // {
//     //     for (auto neighbor : get_reverse_neighbors(root_state))
//     //     {
//     //         Node* root = new Node(State(goal_location, -1,
//     //                 get_direction(neighbor.location,
//     root_state.location)),
//     //                 0, 0, nullptr, 0);
//     //         root->open_handle = heap.push(root);  // add root to heap
//     //         nodes.insert(root);       // add root to hash_table (nodes)
//     //     }
//     // }
//     // else
//     // {
//     Node* root = new Node(goal_location, 0);
//     root->open_handle = heap.push(root);  // add root to heap
//     nodes.insert(root);                   // add root to hash_table (nodes)
//                                           // }

//     while (!heap.empty()) {
//         Node* curr = heap.top();
//         heap.pop();
//         for (auto next_state : instance.graph.getNeighbors(curr->location)) {
//             double curr_weight = instance.graph.getWeight(next_state,
//             curr->location); double next_g_val;
//             // // Add in rotation time
//             // if (consider_rotation)
//             // {
//             //     // Rotating
//             //     if (curr->state.orientation != next_state.orientation)
//             //     {
//             //         int degree =
//             get_rotate_degree(curr->state.orientation,
//             // next_state.orientation);
//             //         next_g_val = curr->g_val +
//             //                      degree * rotation_time * curr_weight;
//             //     }
//             //     // Moving
//             //     else
//             //     {
//             //         next_g_val = curr->g_val + curr_weight;
//             //     }
//             // }
//             // Not considering rotation, only moving
//             // else
//             // {
//             next_g_val = curr->value + curr_weight;
//             // }
//             Node* next = new Node(next_state, next_g_val);
//             auto it = nodes.find(next);
//             if (it == nodes.end()) {  // add the newly generated node to heap
//                                       // and hash table
//                 next->open_handle = heap.push(next);
//                 nodes.insert(next);
//             } else {  // update existing node's g_val if needed (only in the
//                       // heap)
//                 delete (next);  // not needed anymore -- we already generated
//                 it
//                                 // before
//                 Node* existing_next = *it;
//                 if (existing_next->value > next_g_val) {
//                     existing_next->value = next_g_val;
//                     heap.increase(existing_next->open_handle);
//                 }
//             }
//         }
//     }
//     // iterate over all nodes and populate the distances
//     for (auto it = nodes.begin(); it != nodes.end(); it++) {
//         Node* s = *it;
//         my_heuristic[s->location] =
//             std::min(s->value, my_heuristic[s->location]);
//         delete (s);
//     }
//     nodes.clear();
//     heap.clear();
// }

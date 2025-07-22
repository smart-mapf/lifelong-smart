/**
 * This file is the Temporal Plan Graph
 * 
 * Kinodynamic constraints isrepresented as Delta in each node.
 * 
*/
#pragma once

#include <iostream>
#include <vector>
#include <math.h>
#include <memory>
#include <string>


/**
 * The TGP Node is the Temperal reasoning graph
*/
class TPG_Node{
public:
    TPG_Node(std::string name);
    bool SetDelta(double delta);
    bool AddNextNode(std::shared_ptr<TPG_Node> next_node);
    bool AddType2Edge(std::shared_ptr<TPG_Node> next_node);
    double GetDelta();

private:
    std::string node_name;
    double delta;
    std::shared_ptr<TPG_Node> next_node;
    std::shared_ptr<TPG_Node> type2;
};

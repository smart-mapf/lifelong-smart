// #include "/home/admin/Desktop/rpclib/include/rpc/server.h"
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include "rpc/server.h" 


#include <utility>

typedef std::vector<
    std::tuple<std::string, int, double, std::string, std::pair<int, int>, std::pair<double, double>>>
    SIM_PLAN;

SIM_PLAN init(std::string robot_id){
    SIM_PLAN actions;
    actions.push_back(std::make_tuple("00", 0, 0.0, "M",std::make_pair(0, 0), std::make_pair(0.5, 0.0)));
    return actions;

}

int main() {
    rpc::server server(9000);
    server.bind("init", &init);
    server.run();
    return 0;
}
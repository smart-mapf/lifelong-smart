// #include "/home/admin/Desktop/rpclib/include/rpc/server.h"
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include "rpc/server.h" 


#include <utility>



using outputTuple = std::tuple<std::string, int, double, std::string, std::pair<int, int>, std::pair<double, double>>;

std::vector<outputTuple> init(std::string robot_id){
    std::vector<outputTuple> actions;
    actions.push_back(std::make_tuple("00", 0, 0.0, "M",std::make_pair(0, 0), std::make_pair(0.5, 0.0)));
    return actions;

}

int main() {
    rpc::server server(9000);
    server.bind("init", &initActions);
    server.run();
    return 0;
}
#include <arpa/inet.h>
#include <netinet/in.h>
#include <rpc/client.h>
#include <sys/socket.h>
#include <unistd.h>

#include <boost/asio.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <tuple>
#include <vector>
#include <algorithm>

#include <spdlog/spdlog.h>

bool is_port_open(const std::string& ip, int port);
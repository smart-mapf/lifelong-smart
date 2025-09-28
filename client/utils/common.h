#include <arpa/inet.h>
#include <netinet/in.h>
#include <rpc/client.h>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <boost/asio.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "json.hpp"

using json = nlohmann::json;
using std::clamp;
using std::cout;
using std::deque;
using std::endl;
using std::get;
using std::make_pair;
using std::make_shared;
using std::make_tuple;
using std::map;
using std::milli;
using std::min;
using std::pair;
using std::queue;
using std::set;
using std::shared_ptr;
using std::string;
using std::to_string;
using std::tuple;
using std::unordered_set;
using std::vector;

bool is_port_open(const string& ip, int port);
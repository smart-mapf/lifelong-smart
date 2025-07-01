#include "utils/common.h"

bool is_port_open(const std::string& ip, int port) {
    boost::asio::io_context io_context;
    boost::asio::ip::tcp::socket socket(io_context);

    try {
        // Create an endpoint with the provided IP and port
        boost::asio::ip::tcp::endpoint endpoint(
            boost::asio::ip::address::from_string(ip), port);

        // Attempt to connect to the endpoint
        socket.connect(endpoint);
        return true;  // Connection succeeded
    } catch (const boost::system::system_error& e) {
        // Catch connection errors
        // std::cerr << "Connection failed: " << e.what() << std::endl;
        return false;
    }
}

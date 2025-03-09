cmake_minimum_required(VERSION 3.5)
project(my_project)
set(CMAKE_CXX_STANDARD 11)
set(THREADS_PREFER_PTHREAD_FLAG ON)
# Add the path to the rpclib include directory
include_directories(/usr/local/include)
add_subdirectory(externalDependencies/rpclib)
include_directories(./externalDependencies/rpclib/include)
find_package(Threads REQUIRED)
# Add the rpclib library
link_directories(/usr/local/lib)
# Add your executable
add_executable(server mock_adg.cpp)
# Link the rpclib library
target_link_libraries(server rpc)
target_link_libraries(server Threads::Threads)

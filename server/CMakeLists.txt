cmake_minimum_required(VERSION 3.10)

# Project name
project(ADG_Server_Project)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread -g -O3 -ffast-math")
find_package(Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)
SET(CMAKE_BUILD_TYPE "RELEASE")

#include_directories("inc" "/usr/local/include/")
include_directories("inc")

file(GLOB SOURCES "src/*.cpp")

# Add the executable
add_executable(ADG_server ${SOURCES})

# Link libraries
#target_link_libraries(ADG_server PUBLIC ${Boost_LIBRARIES} PRIVATE /usr/local/lib/librpc.a pthread)
target_link_libraries(ADG_server PUBLIC ${Boost_LIBRARIES} PRIVATE rpc pthread)

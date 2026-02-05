cmake_minimum_required(VERSION 3.4)
project(lifelong)

set(CMAKE_CXX_STANDARD 11)

# Find Python 3.8 with development components
find_package(Python 3.8 EXACT COMPONENTS Interpreter Development REQUIRED)
include_directories(${Python_INCLUDE_DIRS})

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RELEASE")
endif()

include_directories("inc")

# Get source files excluding specific ones
file(GLOB SOURCES "src/*.cpp")
list(FILTER SOURCES EXCLUDE REGEX "src/WarehouseSimulation.cpp")
list(FILTER SOURCES EXCLUDE REGEX "src/WarehouseSimulator.cpp")
list(FILTER SOURCES EXCLUDE REGEX "src/WarehouseSimulator.h")

# Find Boost components
find_package(Boost 1.67.0 REQUIRED COMPONENTS program_options system filesystem)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)

# Main executable
add_executable(lifelong ${SOURCES})
target_link_libraries(lifelong PRIVATE ${Boost_LIBRARIES} ${Python_LIBRARIES} rpc pthread spdlog::spdlog)
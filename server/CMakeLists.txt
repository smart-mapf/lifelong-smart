cmake_minimum_required(VERSION 3.10)

# Project name
project(ADG_Server_Project)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set build type to Debug by default if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Add debug flags
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ffast-math -pthread")

find_package(Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)
find_package(spdlog REQUIRED)

include_directories("inc")
include_directories("inc/backup_planners")

file(GLOB SOURCES "src/*.cpp" "src/backup_planners/*.cpp"
                  "src/task_assigners/*.cpp")

# Add the executable
add_executable(ExecutionManager ${SOURCES})

# Link libraries
target_link_libraries(ExecutionManager PUBLIC ${Boost_LIBRARIES} PRIVATE rpc pthread spdlog::spdlog)
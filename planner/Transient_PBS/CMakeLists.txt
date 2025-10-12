project(tpbs)

cmake_minimum_required (VERSION 2.6)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories("inc")
file(GLOB SOURCES "src/*.cpp")
add_executable(tpbs ${SOURCES})

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
find_package(spdlog REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )

# Find Threads
find_package(Threads REQUIRED)          # portable way

target_link_libraries(tpbs ${Boost_LIBRARIES} rpc pthread spdlog::spdlog)

target_compile_options(tpbs PRIVATE -pthread)   # adds -D_REENTRANT automatically

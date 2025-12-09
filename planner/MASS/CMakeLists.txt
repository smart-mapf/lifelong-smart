cmake_minimum_required (VERSION 3.10)
project(MASS)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread -g -ldl -O3 -ffast-math")
set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RELEASE")
endif()

# set base path for cplex directory, change this path if using different version of CPLEX
SET(CPLEX_BASE_DIR "<path to CPLEX>" CACHE STRING "Path to CPLEX base directory")

message(STATUS "CPLEX base directory: ${CPLEX_BASE_DIR}")

set(CPLEX_INCLUDE_DIRS      "${CPLEX_BASE_DIR}/cplex/include")
set(CONCERT_INCLUDE_DIRS      "${CPLEX_BASE_DIR}/concert/include")
set(CPLEX_LIBRARIES         "${CPLEX_BASE_DIR}/cplex/lib/x86-64_linux/static_pic/libcplex.a")
set(ILOCPLEX_LIBRARIES         "${CPLEX_BASE_DIR}/cplex/lib/x86-64_linux/static_pic/libilocplex.a")
# set(CPLEXDISTMIP_LIBRARIES         "${CPLEX_BASE_DIR}/cplex/lib/x86-64_linux/static_pic/libcplexdistmip.a")
set(CONCERT_LIBRARIES         "${CPLEX_BASE_DIR}/concert/lib/x86-64_linux/static_pic/libconcert.a")

find_package(Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)
find_package(spdlog REQUIRED)

include_directories(
  include
)

# add all source files
file(GLOB SOURCES "src/*.cpp")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/)

add_executable(MASS ${SOURCES}
        src/instance.cpp)

set_property(
  TARGET MASS
  PROPERTY COMPILE_DEFINITIONS IL_STD
)

target_include_directories(MASS PUBLIC ${Boost_INCLUDE_DIRS} ${CPLEX_INCLUDE_DIRS} ${CONCERT_INCLUDE_DIRS})
target_link_libraries(MASS ${Boost_LIBRARIES}  ${ILOCPLEX_LIBRARIES} ${CONCERT_LIBRARIES} ${CPLEX_LIBRARIES} ${CPLEXDISTMIP_LIBRARIES}  ${CMAKE_DL_LIBS} rpc spdlog::spdlog)
set (target_link_options "-lilocplex -lconcert -lcplex -lm -lpthread")

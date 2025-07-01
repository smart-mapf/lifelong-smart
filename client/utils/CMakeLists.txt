include_directories(${CMAKE_SOURCE_DIR}/client)
include_directories(${CMAKE_SOURCE_DIR})

find_package(spdlog REQUIRED)

add_library(utils SHARED common.h common.cpp)

target_link_libraries(utils PUBLIC spdlog::spdlog)
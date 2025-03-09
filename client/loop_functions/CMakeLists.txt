include_directories(${CMAKE_SOURCE_DIR}/client)
include_directories(${CMAKE_SOURCE_DIR})


# Descend into the foraging_loop_functions directory

# Descend into the mpga_phototaxis_loop_functions directory
add_subdirectory(mpga_loop_functions)

# Descend into the custom_distributions_loop_functions directory

# If Qt+OpenGL dependencies were found, descend into these directories


# If GALIB was found, also descend into galib_phototaxis_loop_functions


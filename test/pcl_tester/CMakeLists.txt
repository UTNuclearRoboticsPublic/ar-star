cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(greedy_projection)

find_package(PCL 1.2 REQUIRED)
find_package(Eigen REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (greedy_projection greedy_projection.cpp)
target_link_libraries (greedy_projection ${PCL_LIBRARIES})

add_executable (ear_clipping ear_clipping.cpp)
target_link_libraries (ear_clipping ${PCL_LIBRARIES})

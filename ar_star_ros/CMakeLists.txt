cmake_minimum_required(VERSION 3.10.0)
project(ar_star_ros)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(PCL 1.2 REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(lasso_mode src/lasso_mode.cpp)
add_dependencies(lasso_mode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lasso_mode ${catkin_LIBRARIES})
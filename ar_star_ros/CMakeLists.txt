cmake_minimum_required(VERSION 3.10.0)
project(ar_star_ros)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs message_generation)
find_package(PCL 1.2 REQUIRED)
find_package(Eigen REQUIRED)

add_service_files(
  DIRECTORY
  srv
  FILES
  GetPointsInLasso.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS 
  include
  CATKIN_DEPENDS 
  roscpp std_msgs geometry_msgs sensor_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_executable(lasso_listener src/lasso_listener.cpp)
add_dependencies(lasso_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lasso_listener ${catkin_LIBRARIES})

add_executable(points_in_lasso_solver src/points_in_lasso_solver.cpp)
target_link_libraries(points_in_lasso_solver ${catkin_LIBRARIES})
add_dependencies(points_in_lasso_solver ar_star_ros_generate_messages_cpp)
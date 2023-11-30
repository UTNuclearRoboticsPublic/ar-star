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
  GetPointsInHighlight.srv
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

add_executable(highlight_listener src/highlight_listener.cpp)
add_dependencies(highlight_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(highlight_listener ${catkin_LIBRARIES})

add_executable(lasso_listener src/lasso_listener.cpp)
add_dependencies(lasso_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lasso_listener ${catkin_LIBRARIES})

add_executable(lasso_solver src/lasso_solver.cpp)
target_link_libraries(lasso_solver ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(lasso_solver ar_star_ros_generate_messages_cpp)

add_executable(highlight_solver src/highlight_solver.cpp)
target_link_libraries(highlight_solver ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(highlight_solver ar_star_ros_generate_messages_cpp)
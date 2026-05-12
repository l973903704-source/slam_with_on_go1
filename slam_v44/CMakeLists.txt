cmake_minimum_required(VERSION 3.0.2)
project(slam_v44)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs tf visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(slam_v44_core
  src/astar_planner.cpp
  src/laser_processor.cpp
  src/local_controller.cpp
  src/navigation_system.cpp
  src/occupancy_grid_map.cpp
  src/scan_matcher.cpp
  src/slam_system.cpp
)

target_link_libraries(slam_v44_core
  ${catkin_LIBRARIES}
)

add_executable(slam_v44_node
  src/slam_node.cpp
)

target_link_libraries(slam_v44_node
  slam_v44_core
  ${catkin_LIBRARIES}
)

add_executable(slam_v44_navigation_node
  src/navigation_node.cpp
)

target_link_libraries(slam_v44_navigation_node
  slam_v44_core
  ${catkin_LIBRARIES}
)

add_executable(slam_v44_persistent_obstacle_marker_node
  src/obstacle_marker_node.cpp
)

target_link_libraries(slam_v44_persistent_obstacle_marker_node
  ${catkin_LIBRARIES}
)

install(TARGETS slam_v44_core slam_v44_node slam_v44_navigation_node slam_v44_persistent_obstacle_marker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


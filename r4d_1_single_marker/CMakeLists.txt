cmake_minimum_required(VERSION 2.8.3)
project(r4d_1_single_marker)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  r4d_common
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES r4d_1_single_marker
#  CATKIN_DEPENDS geometry_msgs nav_msgs r4d_common rcars_detector_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(single_marker_node src/single_marker_node.cpp)

target_link_libraries(single_marker_node
  ${catkin_LIBRARIES}
)

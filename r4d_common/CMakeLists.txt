cmake_minimum_required(VERSION 2.8.3)
project(r4d_common)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  std_msgs
  rosbag
  tf
  eigen_conversions
  rcars_detector_msgs
  gtsam_catkin
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES r4d_common
  CATKIN_DEPENDS geometry_msgs nav_msgs rcars_detector_msgs std_msgs rosbag tf eigen_conversions gtsam_catkin
  DEPENDS OpenCV Eigen3
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}	
  ${catkin_INCLUDE_DIRS}
)

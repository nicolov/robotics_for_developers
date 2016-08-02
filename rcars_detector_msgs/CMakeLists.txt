cmake_minimum_required(VERSION 2.8.3)
project(rcars_detector_msgs)

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3 -march=native")

set(${CMAKE_BUILD_TYPE} RELEASE)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(FILES
  Tag.msg
  TagArray.msg
  TagPoses.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   INCLUDE_DIRS
   CATKIN_DEPENDS geometry_msgs
   CATKIN_DEPENDS message_runtime
)


include_directories(
   ${catkin_INCLUDE_DIRS}
)

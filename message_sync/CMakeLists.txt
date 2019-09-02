cmake_minimum_required(VERSION 2.8.3)
project(message_sync)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  message_filters
  yidamsg
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS roscpp)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

add_executable(message_sync src/message_sync.cpp)
target_link_libraries(message_sync ${catkin_LIBRARIES})
cmake_minimum_required(VERSION 2.8.3)
project(ros_daemon)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  yidamsg
)

catkin_package(CATKIN_DEPENDS roscpp)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ros_daemon src/ros_daemon.cpp)
add_dependencies(ros_daemon ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_daemon
     ${catkin_LIBRARIES})
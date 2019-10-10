cmake_minimum_required(VERSION 2.8.3)
project(move_base_client)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  genmsg
)

SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

include_directories(include ${catkin_INCLUDE_DIRS})

 add_service_files(
   FILES
   wali_go_to_position.srv
 )

  generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(INCLUDE_DIRS include
              CATKIN_DEPENDS roscpp)

add_executable(navigation src/navigation.cpp)
add_dependencies(navigation ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS})
target_link_libraries(navigation ${catkin_LIBRARIES})
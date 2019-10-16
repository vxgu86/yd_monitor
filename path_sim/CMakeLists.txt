cmake_minimum_required(VERSION 2.8.3)
project(path_sim)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  rospy
  std_msgs
  yidamsg
)
find_package(PCL 1.7 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})


add_executable(plan_sim src/plan_sim.cpp src/Vect.cpp src/utils.cpp)
target_link_libraries(plan_sim
    ${CORE_LIBRARY} 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(path_planning src/path_planning.cpp src/Vect.cpp)
target_link_libraries(path_planning
    ${CORE_LIBRARY} 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
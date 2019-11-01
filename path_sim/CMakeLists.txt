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
   INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
LIBRARIES
        stp
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_library(c3_algorithm
src/c3_algorithm.cpp
src/c2_algorithm.cpp
)


include_directories(${PROJECT_SOURCE_DIR}/include 
${catkin_INCLUDE_DIRS} 
${PCL_INCLUDE_DIRS}
)


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

#add_executable(smooth_vel src/smooth_vel.cpp )
#add_dependencies(smooth_vel ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS})
#target_link_libraries(smooth_vel
#    ${CORE_LIBRARY} 
#    ${catkin_LIBRARIES}
#    ${PCL_LIBRARIES}
#)

add_executable(c3 src/c3.cpp)
add_dependencies(c3 ${catkin_EXPORTED_TARGETS} ${catkin_INCLUDE_DIRS})
target_link_libraries(c3
    c3_algorithm
    ${CORE_LIBRARY} 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

#add_executable(test_angular src/test_angular.cpp)
#target_link_libraries(test_angular
#    ${CORE_LIBRARY} 
#    ${catkin_LIBRARIES}
#    ${PCL_LIBRARIES}
#)

#add_executable(test_velocity src/test_velocity.cpp)
#target_link_libraries(test_velocity
#    ${CORE_LIBRARY} 
#    ${catkin_LIBRARIES}
#    ${PCL_LIBRARIES}
#)
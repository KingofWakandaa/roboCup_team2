cmake_minimum_required(VERSION 2.8.3)
project(place)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  geometry_msgs
  tf2_geometry_msgs
  actionlib
  play_motion_msgs
  trajectory_msgs
)

catkin_package(
INCLUDE_DIRS include
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(placethings src/placethings.cpp)
add_dependencies(placethings ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(placethings
   ${catkin_LIBRARIES}
 )


install(TARGETS placethings
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

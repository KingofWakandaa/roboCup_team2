cmake_minimum_required(VERSION 2.8.3)
project(grasp)

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

add_executable(takethings src/takethings.cpp src/Arm_class.cpp)
add_dependencies(takethings ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(takethings
   ${catkin_LIBRARIES}
 )

#add_executable(plan_arm_torso_ik src/plan_arm_torso_ik.cpp)
# add_dependencies(plan_arm_torso_ik ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(plan_arm_torso_ik
#    ${catkin_LIBRARIES}
#  )
add_executable(placethings src/placethings.cpp src/Arm_class.cpp)
add_dependencies(placethings ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(placethings
   ${catkin_LIBRARIES}
 )


install(TARGETS placethings
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
install(TARGETS takethings
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

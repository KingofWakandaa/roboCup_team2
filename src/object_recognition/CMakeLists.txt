cmake_minimum_required(VERSION 2.8.3)
project(object_recognition)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  tf
  image_geometry
  darknet_ros_msgs
  perception_msgs
#>>>>TODO: Add all the dependencies you need. Don't forget to update also the package manifest (package.xml).

)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
INCLUDE_DIRS include
LIBRARIES object_recognition
CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf image_geometry
)
## Specify additional locations of header files
## Your package locations should be listed before other locations

#>>>>TODO: Add the other include locations.
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${PCL_INCLUDE_DIRS}
 
)

#add_library(plane_segmentation_tutorial src/PlaneSegmentation.cpp)
#target_link_libraries(plane_segmentation_tutorial ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#>>>>TODO: Create a library for the class declared in the files.

#>>>>TODO: Link the executable with the library.


## Declare a cpp executable
#add_library(ObjectRecognition src/ObjectRecognition.cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(ObjectRecognition ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

#add_dependencies(ObjectRecognition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#add_executable(node_objectRecognition  src/node_objectRecognition .cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(node_objectRecognition  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

#add_dependencies(node_objectRecognition  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(ObjectRecognition src/ObjectRecognition.cpp)
target_link_libraries(ObjectRecognition
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(ObjectRecognition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(node_objectRecognition src/node_objectRecognition.cpp)
target_link_libraries(node_objectRecognition
  
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ObjectRecognition
)
add_dependencies(node_objectRecognition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
install(TARGETS node_objectRecognition
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

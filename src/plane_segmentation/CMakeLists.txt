cmake_minimum_required(VERSION 2.8.3)
project(plane_segmentation)


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

#>>>>TODO: Add all the dependencies you need. Don't forget to update also the package manifest (package.xml).

)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
INCLUDE_DIRS include
LIBRARIES plane_segmentation
CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf image_geometry
)
## Specify additional locations of header files
## Your package locations should be listed before other locations

#>>>>TODO: Add the other include locations.
include_directories(
  ${catkin_INCLUDE_DIRS}  
  ${PCL_INCLUDE_DIRS}
  include
)

add_library(plane_segmentation_tutorial src/PlaneSegmentation.cpp)
target_link_libraries(plane_segmentation_tutorial ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#>>>>TODO: Create a library for the class declared in the files.

#>>>>TODO: Link the executable with the library.


## Declare a cpp executable
add_executable(segment_node src/segment_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(segment_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} plane_segmentation_tutorial)





cmake_minimum_required(VERSION 2.8.3)
project(callisto_linefollower)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rospy
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
message( STATUS "OpenCV Include Dir: " ${OpenCV_INCLUDE_DIRS} )


add_executable(image_republisher src/imagerepublisher.cpp)
target_link_libraries(image_republisher ${catkin_LIBRARIES})
add_dependencies(image_republisher callisto_linefollower_msgs_gencpp)


find_package(OpenCV 3.2.0 EXACT REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(image_republisher ${OpenCV_LIBRARIES})

find_package(cv_bridge)
include_directories(${cv_bridge_INCLUDE_DIRS})
target_link_libraries(image_republisher ${cv_bridge_LIBRARIES})

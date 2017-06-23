cmake_minimum_required(VERSION 2.8.3)
project(split_and_merge)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
)

catkin_package(
 INCLUDE_DIRS
  include
 CATKIN_DEPENDS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_executable(odometry_node
               src/odometry.cpp)

add_dependencies(odometry_node split_and_merge_generate_messages_cpp)

target_link_libraries(odometry_node
   ${catkin_LIBRARIES}
)

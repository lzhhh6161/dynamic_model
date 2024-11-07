cmake_minimum_required(VERSION 3.0.2)
project(dynamic_model)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  tf2_ros
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(obstaclePathPlugin src/obstaclePathPlugin.cc)

set_target_properties(obstaclePathPlugin
    PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

target_link_libraries(obstaclePathPlugin    
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
)

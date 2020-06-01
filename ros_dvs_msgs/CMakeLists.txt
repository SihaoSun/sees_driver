cmake_minimum_required(VERSION 2.8.3)
project(ros_dvs_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
)

FILE(GLOB sees_message_definitions RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

add_message_files(
  FILES
  ${sees_message_definitions}
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

cmake_minimum_required(VERSION 2.8.3)
project(ros_common_time)

set(${PROJECT_NAME}_PKG_DEPENDENCIES
  roscpp
)

add_compile_options(
  -std=c++11
  -O3
)

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_PKG_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ${${PROJECT_NAME}_PKG_DEPENDENCIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY include/iness_common
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

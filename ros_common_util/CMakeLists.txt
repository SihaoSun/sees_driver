cmake_minimum_required(VERSION 2.8.3)
project(ros_common_util)

set(${PROJECT_NAME}_PKG_DEPENDENCIES
  cmake_modules
  sees
  roscpp
  cv_bridge
  ros_common_time
)

add_compile_options(
  -std=c++11
  -O3
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_PKG_DEPENDENCIES}
)


catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ${${PROJECT_NAME}_PKG_DEPENDENCIES}
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} STATIC
    src/opencv_image_publisher.cpp
)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

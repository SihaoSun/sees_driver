cmake_minimum_required(VERSION 2.8.3)
project(ros_sees)

set(${PROJECT_NAME}_PKG_DEPENDENCIES
  roscpp
  sees
  ros_dvs_msgs
  dynamic_reconfigure
  ros_common_util
  eigen_catkin
  sensor_msgs
  rpg_common
)

SET(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo

add_compile_options(
  -std=c++11
  -O3
)
find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_PKG_DEPENDENCIES}
)

find_package(eigen_catkin REQUIRED)

# configure ROS dynamic_reconfigure
generate_dynamic_reconfigure_options(
  config/ros_sees.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ${${PROJECT_NAME}_PKG_DEPENDENCIES}
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(LIBRARY_CPP_FILES
  src/ros_sees_publisher.cpp
  src/ros_sees_publisher_mod.cpp
  src/stereo_class.cpp
)

add_executable(sees_node
  src/main.cpp
)

add_executable(sees_node_stereo
  src/stereo.cpp
)

add_executable(sees_node_stereo_class
  src/stereo_class
)

add_dependencies(sees_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(sees_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_dependencies(sees_node_stereo
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(sees_node_stereo
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_dependencies(sees_node_stereo_class
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(sees_node_stereo_class
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} STATIC
  ${LIBRARY_CPP_FILES}
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

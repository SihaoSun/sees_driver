cmake_minimum_required (VERSION 2.8.3)
project(sees)


find_package(catkin)

find_library(sees_LIBRARIES NAMES ${PROJECT_NAME} PATHS "lib")

catkin_package(
    INCLUDE_DIRS
      include
    CATKIN_DEPENDS
    LIBRARIES
      ${sees_LIBRARIES}
)

include_directories(include ${catkin_INCLUDE_DIRS})

install(DIRECTORY lib/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)



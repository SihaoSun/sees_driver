cmake_minimum_required (VERSION 2.8.3)
project(sees)

find_package(catkin)

# set architecture type in the lib path (armv7l, linux64, aarch64)
set(PATH_TO_LIB "lib/aarch64")

find_library(sees_LIBRARIES NAMES iness libiness ${PROJECT_NAME} PATHS ${PATH_TO_LIB})

catkin_package(
    INCLUDE_DIRS
      include
    CATKIN_DEPENDS
    LIBRARIES
      ${sees_LIBRARIES}
)
add_compile_options(
  -std=c++14
)
  
include_directories(include ${catkin_INCLUDE_DIRS})

install(DIRECTORY lib/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)



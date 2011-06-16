cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

#search MAVLINK and glib library
SET(
  CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake
)
include(cmake/FindMAVLINK.cmake)
include(cmake/FindGLIB2.cmake)
include(cmake/FindGTHREAD2.cmake)

FIND_PACKAGE(MAVLINK  REQUIRED)
FIND_PACKAGE(GLIB2  REQUIRED)
FIND_PACKAGE(GTHREAD2  REQUIRED)

INCLUDE_DIRECTORIES(
  ${MAVLINK_INCLUDE_DIR}
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
  ../include/
)

rosbuild_add_executable(mavlink_bridge_serial src/mavlink_ros_serial.cpp)
target_link_libraries(mavlink_bridge_serial
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
)

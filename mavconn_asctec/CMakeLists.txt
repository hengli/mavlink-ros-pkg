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

#rosbuild_genmsg()

#Load MAVCONN specific libraries (for dependencies please check the MAVCONN installation tutorial)
SET(
  CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake
)
include(cmake/FindMAVCONN.cmake)
include(cmake/FindMAVLINK.cmake)
include(cmake/FindGLIB2.cmake)

FIND_PACKAGE(MAVCONN REQUIRED)
FIND_PACKAGE(MAVLINK REQUIRED)
FIND_PACKAGE(GLIB2 REQUIRED)

INCLUDE_DIRECTORIES(
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${MAVCONN_INCLUDE_DIR}
  ${MAVLINK_INCLUDE_DIR}
  ../include/
)

rosbuild_add_executable(mavconn_asctec_bridge src/mavconn_asctec_bridge.cpp)
target_link_libraries(mavconn_asctec_bridge
  mavconn_lcm
)

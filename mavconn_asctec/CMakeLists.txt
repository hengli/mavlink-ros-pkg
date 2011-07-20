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

SET(PKG asctec_hl_comm)
EXECUTE_PROCESS(COMMAND rospack find ${PKG}
                OUTPUT_VARIABLE PKG_PATH)

IF (NOT ${PKG_PATH} STREQUAL "")

EXECUTE_PROCESS(COMMAND rospack cflags-only-I ${PKG}
                OUTPUT_VARIABLE PKG_INCLUDE_DIRS
		OUTPUT_STRIP_TRAILING_WHITESPACE)
EXECUTE_PROCESS(COMMAND rospack libs-only-L ${PKG}
                OUTPUT_VARIABLE PKG_LIBRARY_DIRS
		OUTPUT_STRIP_TRAILING_WHITESPACE)
EXECUTE_PROCESS(COMMAND rospack libs-only-l -q ${PKG}
                OUTPUT_VARIABLE PKG_LIBRARIES
		OUTPUT_STRIP_TRAILING_WHITESPACE)
string(REPLACE " " ";" PKG_INCLUDE_DIRS ${PKG_INCLUDE_DIRS})
string(REPLACE " " ";" PKG_LIBRARY_DIRS ${PKG_LIBRARY_DIRS})
string(REPLACE " " ";" PKG_LIBRARIES ${PKG_LIBRARIES})

INCLUDE_DIRECTORIES(${PKG_INCLUDE_DIRS})

rosbuild_add_executable(mavconn_asctec_bridge src/mavconn_asctec_bridge.cpp)

LINK_DIRECTORIES(
  ${PKG_LIBRARY_DIRS}
)
TARGET_LINK_LIBRARIES(mavconn_asctec_bridge
  ${PKG_LIBRARIES}
  mavconn_lcm
)

ENDIF (NOT ${PKG_PATH} STREQUAL "")


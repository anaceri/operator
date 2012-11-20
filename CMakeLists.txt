# CMake file to compile operator ros node

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#### Include & Libs #####

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

link_directories(/lib)
include_directories(include)
rosbuild_add_executable(operator src/operator.cpp include/dhdc.h include/drdc.h src/UDPconnect.h src/controlfunctions.h src/controlfunctions.cpp src/CConstants.h src/CMaths.h src/CMaths.cpp src/CVector3d.h src/CMatrix3d.h)

find_library(LIB_DHD dhd lib)
find_library(LIB_DRD drd lib)
target_link_libraries(operator ${LIB_DHD} ${LIB_DRD} pthread usb-1.0 rt)



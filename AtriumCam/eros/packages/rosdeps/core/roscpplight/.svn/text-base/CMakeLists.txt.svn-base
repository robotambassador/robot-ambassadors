cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(example src/example/example.cpp)
rosbuild_add_executable(example src/roscpplight/nodehandle.cpp)
rosbuild_add_executable(example src/roscpplight/nodexmlrpcclient.cpp)
rosbuild_add_executable(example src/roscpplight/nodexmlrpcserver.cpp)
rosbuild_add_executable(example src/roscpplight/nodexmlrpcparser.cpp)
rosbuild_add_executable(example src/roscpplight/headerpacket.cpp)

rosbuild_link_boost(example regex)
rosbuild_link_boost(example system)

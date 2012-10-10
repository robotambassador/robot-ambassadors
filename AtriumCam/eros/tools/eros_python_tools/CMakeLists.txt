cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(CMAKE_INSTALL_PREFIX $ENV{ROS_ROOT} CACHE PATH "Install location" FORCE)

add_subdirectory(scripts)

##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.4.6)

##############################################################################
# Init Configuration
##############################################################################

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_include(eros_build eros_package)
eros_3rd_party_init()

##############################################################################
# Variables
##############################################################################

include(${CMAKE_SOURCE_DIR}/cmake/OpenCvBuild.cmake)

##############################################################################
# Installation
##############################################################################
#
# OPENCV_INSTALL_DIR was set in OpenCvBuild.cmake (CMAKE_SOURCE_DIR/opencv)

install(DIRECTORY ${OPENCV_INSTALL_DIR}/include/ DESTINATION include USE_SOURCE_PERMISSIONS)
install(DIRECTORY ${OPENCV_INSTALL_DIR}/lib/ DESTINATION lib USE_SOURCE_PERMISSIONS)
install(DIRECTORY ${OPENCV_INSTALL_DIR}/bin/ DESTINATION bin USE_SOURCE_PERMISSIONS)
install(DIRECTORY ${OPENCV_INSTALL_DIR}/share/ DESTINATION share USE_SOURCE_PERMISSIONS)

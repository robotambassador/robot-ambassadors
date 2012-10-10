##############################################################################
# Cmake
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

set(SOURCE_NAME "zlib-1.2.5")
set(SOURCE_TARBALL "${SOURCE_NAME}.tar.gz")
set(SOURCE_TARBALL_URL "http://zlib.net/${SOURCE_TARBALL}")
set(SOURCE_DIR ${CMAKE_BINARY_DIR}/${SOURCE_NAME})
set(PREFIX ${CMAKE_BINARY_DIR}/fakeroot)

set(COMMON_ARGS --prefix=${PREFIX} --eprefix=${PREFIX})
set(CONFIGURE_COMMAND  CC=${CMAKE_C_COMPILER} ./configure ${COMMON_ARGS})

##############################################################################
# Build
##############################################################################

eros_download(${SOURCE_TARBALL_URL} ${CMAKE_BINARY_DIR}/${SOURCE_TARBALL})
eros_extract_tarball(${CMAKE_BINARY_DIR}/${SOURCE_TARBALL} ${CMAKE_BINARY_DIR})
eros_autotools_compile(CONFIGURE_COMMAND ${SOURCE_DIR} ${CMAKE_BINARY_DIR}/extracted)

add_custom_target(
        source ALL
        DEPENDS ${SOURCE_DIR}/compiled
        COMMENT "This is ${PROJECT_NAME}."
        )

##############################################################################
# Install
##############################################################################

install(DIRECTORY ${PREFIX}/include/
        DESTINATION include
        USE_SOURCE_PERMISSIONS
        )

install(DIRECTORY ${PREFIX}/lib/
        DESTINATION lib
        USE_SOURCE_PERMISSIONS
        )

install(DIRECTORY ${PREFIX}/share/
        DESTINATION share
        USE_SOURCE_PERMISSIONS
        )


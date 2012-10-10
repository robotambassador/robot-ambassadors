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
# Abort Checks
##############################################################################

if(NOT CMAKE_CROSSCOMPILING)
  message(FATAL "This package is only intended for cross-compilations.")
endif()
# This needs TOOLCHAIN_TUPLE and TOOLCHAIN_SYSROOT variables defined by eros toolchains.
if(NOT DEFINED TOOLCHAIN_TUPLE)
  message(FATAL_ERROR "${TOOLCHAIN_TUPLE} is undefined - this script needs an eros toolchain to extract this information.")
endif()
if(NOT DEFINED TOOLCHAIN_SYSROOT)
  message(FATAL_ERROR "${TOOLCHAIN_SYSROOT} is undefined - this script needs an eros toolchain to extract this information.")
endif()

##############################################################################
# Variables
##############################################################################

set(SOURCE_NAME "gdb-7.1")
set(SOURCE_TARBALL "${SOURCE_NAME}.tar.gz")
set(SOURCE_TARBALL_URL "http://ftp.gnu.org/gnu/gdb/${SOURCE_TARBALL}")
set(SOURCE_DIR ${CMAKE_BINARY_DIR}/${SOURCE_NAME})

set(PREFIX ${CMAKE_BINARY_DIR}/fakeroot)
set(LOCAL_PREFIX ${CMAKE_BINARY_DIR}/localroot)
set(CMAKE_LOCAL_INSTALL_PREFIX /usr/local)

set(GDB_CONFIG_FLAGS --disable-werror --with-build-sysroot=${TOOLCHAIN_SYSROOT} --target=${TOOLCHAIN_TUPLE})
set(CONFIGURE_GDB_COMMAND ./configure ${GDB_CONFIG_FLAGS} --prefix=${LOCAL_PREFIX} --exec-prefix=${LOCAL_PREFIX})
set(CONFIGURE_GDB_SERVER_COMMAND ./configure --prefix=${PREFIX} --exec-prefix=${PREFIX} --host=${TOOLCHAIN_TUPLE} --target=${TOOLCHAIN_TUPLE} CC=${CMAKE_C_COMPILER} CXX=${CMAKE_CXX_COMPILER})

##############################################################################
# Download
##############################################################################

eros_download(${SOURCE_TARBALL_URL} ${CMAKE_BINARY_DIR}/${SOURCE_TARBALL})
eros_extract_tarball(${CMAKE_BINARY_DIR}/${SOURCE_TARBALL} ${CMAKE_BINARY_DIR})

##############################################################################
# Building
##############################################################################

set(COMPILED_FLAG ${CMAKE_BINARY_DIR}/compiled)
set(GDB_COMPILED_FLAG ${CMAKE_BINARY_DIR}/gdb_compiled)

add_custom_command(OUTPUT ${GDB_COMPILED_FLAG}
    COMMAND echo ${CONFIGURE_GDB_COMMAND}
    COMMAND ${CONFIGURE_GDB_COMMAND}
    COMMAND make $ENV{ROS_PARALLEL_JOBS}
    COMMAND make install
    COMMAND touch ${GDB_COMPILED_FLAG}
    DEPENDS ${CMAKE_BINARY_DIR}/extracted
    WORKING_DIRECTORY ${SOURCE_DIR}
    COMMENT "Compiling gdb."
    VERBATIM
    )

add_custom_command(OUTPUT ${COMPILED_FLAG}
    COMMAND echo ${CONFIGURE_GDB_SERVER_COMMAND}
    COMMAND ${CONFIGURE_GDB_SERVER_COMMAND}
    COMMAND make $ENV{ROS_PARALLEL_JOBS}
    COMMAND make install
    COMMAND touch ${COMPILED_FLAG}
    DEPENDS ${GDB_COMPILED_FLAG}
    WORKING_DIRECTORY ${SOURCE_DIR}/gdb/gdbserver
    COMMENT "Compiling gdbserver."
    VERBATIM
    )

# By adding ALL here, it automatically gets built before the packaging command.
add_custom_target(
        source ALL
        DEPENDS ${COMPILED_FLAG}
        COMMENT "This is ${PROJECT_NAME}."
        )

##############################################################################
# Install
##############################################################################

install(DIRECTORY ${LOCAL_PREFIX}/bin/
        DESTINATION ${CMAKE_LOCAL_INSTALL_PREFIX}/bin
        USE_SOURCE_PERMISSIONS
        )

install(DIRECTORY ${LOCAL_PREFIX}/lib/
        DESTINATION ${CMAKE_LOCAL_INSTALL_PREFIX}/lib
        USE_SOURCE_PERMISSIONS
        )

install(DIRECTORY ${LOCAL_PREFIX}/share/
        DESTINATION ${CMAKE_LOCAL_INSTALL_PREFIX}/share
        USE_SOURCE_PERMISSIONS
        )

install(DIRECTORY ${PREFIX}/bin/
        DESTINATION bin
        USE_SOURCE_PERMISSIONS
        )

install(DIRECTORY ${PREFIX}/share/
        DESTINATION share
        USE_SOURCE_PERMISSIONS
        )

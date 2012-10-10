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

set(CMAKE_VERBOSE_MAKEFILE true)

##############################################################################
# Variables
##############################################################################

set(SOURCE_NAME "apr-1.4.2")
set(SOURCE_TARBALL "${SOURCE_NAME}.tar.gz")
#set(SOURCE_TARBALL_URL "http://mirror.apache-kr.org/apr/${SOURCE_TARBALL}")
# New version out, using the archive for now.
set(SOURCE_TARBALL_URL "http://archive.apache.org/dist/apr/${SOURCE_TARBALL}")

set(SOURCE_DIR ${CMAKE_BINARY_DIR}/${SOURCE_NAME})

# The compilers and ros compile flags are all set by rostoolchain.cmake and rosconfig.cmake
if(MINGW)
  # Fix a bug in mingw cross compiler not setting this macro for you.
  set(ROS_COMPILE_FLAGS "${ROS_COMPILE_FLAGS} -D_WIN32_WINNT=0x0500")
endif()

set(COMMON_ARGS --prefix=${CMAKE_INSTALL_PREFIX} 
                --exec-prefix=${CMAKE_INSTALL_PREFIX} 
                C=${CMAKE_C_COMPILER} 
                CXX=${CMAKE_CXX_COMPILER} 
                CFLAGS=${ROS_COMPILE_FLAGS} 
                CXXFLAGS=${ROS_COMPILE_FLAGS}
                LDFLAGS=${ROS_LINK_FLAGS}
                )

if(ROS_BUILD_STATIC_LIBS)
  set(PACKAGE_OPTIONS --disable-shared --enable-static)
else()
  set(PACKAGE_OPTIONS --disable-static --enable-shared)
endif()

if(CMAKE_CROSSCOMPILING)
  if(MINGW)
    set(PRESET_VARIABLES 
        # Sometimes wine has an emulation environment which runs conf tests,
        # unfortunately these return like a 4^M, not 4 which breaks all the 
        # configure script logic.
        ac_cv_sizeof_off_t=4
        ac_cv_sizeof_pid_t=4 
        ac_cv_sizeof_size_t=4
        ac_cv_sizeof_ssize_t=4
        )
  else() # Arm
    # Might need a check to properly evaluate ac_cv_sizeof_struct_iovec
    set(PRESET_VARIABLES ac_cv_file__dev_zero=yes ac_cv_func_setpgrp_void=yes apr_cv_tcp_nodelay_with_cork=no apr_cv_process_shared_works=no apr_cv_mutex_robust_shared=no ac_cv_sizeof_struct_iovec=8)
  endif()
  set(CONFIGURE_COMMAND ./configure --host=${TOOLCHAIN_TUPLE} --target=${TOOLCHAIN_TUPLE} ${PACKAGE_OPTIONS} ${PRESET_VARIABLES} ${COMMON_ARGS}) 
else()
  set(CONFIGURE_COMMAND ./configure ${PACKAGE_OPTIONS} ${COMMON_ARGS})
endif()

##############################################################################
# Build
##############################################################################

eros_download(${SOURCE_TARBALL_URL} ${CMAKE_BINARY_DIR}/${SOURCE_TARBALL})
eros_extract_tarball(${CMAKE_BINARY_DIR}/${SOURCE_TARBALL} ${CMAKE_BINARY_DIR})
eros_autotools_compile_only(CONFIGURE_COMMAND ${SOURCE_DIR} ${CMAKE_BINARY_DIR}/extracted)

add_custom_target(
        source ALL
        DEPENDS ${SOURCE_DIR}/compiled
        COMMENT "This is ${PROJECT_NAME}."
        )

##############################################################################
# Install
##############################################################################

add_custom_target(
        custom_install
        make install
        DEPENDS ${SOURCE_DIR}/compiled
        WORKING_DIRECTORY ${SOURCE_DIR}
        COMMENT "Installing ${PROJECT_NAME}."
        )

# Doesn't have its own uninstall command!
add_custom_target(
        custom_uninstall
        rm -f ${CMAKE_INSTALL_PREFIX}/bin/apr-1-config
        COMMAND rm -rf ${CMAKE_INSTALL_PREFIX}/include/apr-1
        COMMAND rm -rf ${CMAKE_INSTALL_PREFIX}/build-1
        COMMAND rm -f ${CMAKE_INSTALL_PREFIX}/lib/libapr-1*
        COMMAND rm -f ${CMAKE_INSTALL_PREFIX}/lib/apr.exp
        COMMAND rm -rf ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/apr-1.pc
        DEPENDS ${SOURCE_DIR}/compiled
        WORKING_DIRECTORY ${SOURCE_DIR}
        COMMENT "Uninstalling ${PROJECT_NAME}."
        )
        

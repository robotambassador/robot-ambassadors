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

set(SOURCE_NAME "apache-log4cxx-0.10.0")
set(SOURCE_TARBALL "apache-log4cxx-0.10.0.tar.gz")
#set(SOURCE_TARBALL_URL "http://apache.naggo.co.kr//logging/log4cxx/0.10.0/${SOURCE_TARBALL}")
set(SOURCE_TARBALL_URL "http://apache.mirror.cdnetworks.com/logging/log4cxx/0.10.0/${SOURCE_TARBALL}")
set(SOURCE_DIR ${CMAKE_BINARY_DIR}/${SOURCE_NAME})

if(MINGW)                    
  # Fix a bug in mingw cross compiler not setting this macro for you.
  set(ROS_COMPILE_FLAGS "${ROS_COMPILE_FLAGS} -D_WIN32_WINNT=0x0500")
endif()

if(ROS_BUILD_STATIC_LIBS)
  set(PLATFORM_OPTIONS --disable-shared --enable-static)
else()
  set(PLATFORM_OPTIONS --disable-static --enable-shared)
endif()

if(CMAKE_CROSSCOMPILING)
  if(MINGW)
    set(PRESET_VARIABLES ) # Nothing here
  else() # Arm
    # Might need a check to properly evaluate ac_cv_sizeof_struct_iovec
    set(PRESET_VARIABLES ac_cv_file__dev_zero=yes ac_cv_func_setpgrp_void=yes apr_cv_tcp_nodelay_with_cork=no apr_cv_process_shared_works=no apr_cv_mutex_robust_shared=no ac_cv_sizeof_struct_iovec=8)
  endif()
  set(PLATFORM_OPTIONS ${PLATFORM_OPTIONS} --host=${TOOLCHAIN_TUPLE} --target=${TOOLCHAIN_TUPLE}) 
else()
endif()

set(COMMON_ARGS --prefix=${CMAKE_INSTALL_PREFIX} 
                --exec-prefix=${CMAKE_INSTALL_PREFIX} 
                --with-apr=${CMAKE_INSTALL_PREFIX}
                --with-apr-util=${CMAKE_INSTALL_PREFIX}
                C=${CMAKE_C_COMPILER} 
                CXX=${CMAKE_CXX_COMPILER}
                CFLAGS=${ROS_COMPILE_FLAGS} 
                CXXFLAGS=${ROS_COMPILE_FLAGS}
                LDFLAGS=${ROS_LINK_FLAGS}
                )
set(CONFIGURE_COMMAND ./configure  ${PLATFORM_OPTIONS} ${PRESET_VARIABLES} ${COMMON_ARGS}) 

##############################################################################
# DOWNLOADING/GENERATING THE SOURCES
##############################################################################

set(PATCHED_FLAG ${CMAKE_BINARY_DIR}/patched)

eros_download(${SOURCE_TARBALL_URL} ${CMAKE_BINARY_DIR}/${SOURCE_TARBALL})
eros_extract_tarball(${CMAKE_BINARY_DIR}/${SOURCE_TARBALL} ${CMAKE_BINARY_DIR})

set(PATCH_THREAD_COMMAND_ONE patch -p1 < ${CMAKE_SOURCE_DIR}/patches/log4cxx-1-gcc.4.4.patch)
set(PATCH_THREAD_COMMAND_TWO patch -p1 < ${CMAKE_SOURCE_DIR}/patches/log4cxx-2-pkgconfig.patch)
set(PATCH_THREAD_COMMAND_THREE patch -p1 < ${CMAKE_SOURCE_DIR}/patches/log4cxx-3-warnings.patch)
set(PATCH_THREAD_COMMAND_FOUR patch -p1 < ${CMAKE_SOURCE_DIR}/patches/log4cxx-4-mingw.definitions.patch)

# Patch
add_custom_command(OUTPUT ${PATCHED_FLAG}
    COMMAND ${PATCH_THREAD_COMMAND_ONE}
    COMMAND ${PATCH_THREAD_COMMAND_TWO}
    COMMAND ${PATCH_THREAD_COMMAND_THREE}
    COMMAND ${PATCH_THREAD_COMMAND_FOUR}
    COMMAND touch ${PATCHED_FLAG}
    DEPENDS ${CMAKE_BINARY_DIR}/extracted
    WORKING_DIRECTORY ${SOURCE_DIR}
    COMMENT "Patching ${SOURCE_DIR}."
    VERBATIM
    )

##############################################################################
# Build
##############################################################################

eros_autotools_compile_only(CONFIGURE_COMMAND ${SOURCE_DIR} ${PATCHED_FLAG})

# By adding ALL here, it automatically gets built before the packaging command.
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

add_custom_target(
        custom_uninstall
        make install
        DEPENDS ${SOURCE_DIR}/compiled
        WORKING_DIRECTORY ${SOURCE_DIR}
        COMMENT "Uninstalling ${PROJECT_NAME}."
        )
       


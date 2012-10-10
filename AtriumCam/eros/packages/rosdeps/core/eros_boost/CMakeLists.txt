##############################################################################
# Cmake
##############################################################################
#
# This builds a minimal boost intended for embedded targets
# 
# Options: without-icu (in regex)
# Libraries needed by various components 
#   ros bootstrap : date_time, filesystem, regex, signals, thread
#   ycs           : program_options
#
cmake_minimum_required(VERSION 2.4.6)

##############################################################################
# Init Configuration
##############################################################################

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_include(eros_build eros_package)
eros_3rd_party_init()
eros_detect_compiler_version() # gets the COMPILER_VERSION variable (used by bjam)

##############################################################################
# Variables
##############################################################################

set(SOURCE_VERSION_MAJOR "1")
set(SOURCE_VERSION_MINOR "40")
set(SOURCE_VERSION_PATCH "0")
set(SOURCE_VERSION "${SOURCE_VERSION_MAJOR}.${SOURCE_VERSION_MINOR}.${SOURCE_VERSION_PATCH}")
set(SOURCE_VERSION_UNDERSCORE "${SOURCE_VERSION_MAJOR}_${SOURCE_VERSION_MINOR}_${SOURCE_VERSION_PATCH}")
set(SOURCE_NAME "boost-${SOURCE_VERSION}")
set(SOURCE_TARBALL "boost_${SOURCE_VERSION_UNDERSCORE}.tar.gz")
set(SOURCE_TARBALL_URL "http://sourceforge.net/projects/boost/files/boost/${SOURCE_VERSION}/${SOURCE_TARBALL}/download")
set(SOURCE_DIR boost_${SOURCE_VERSION_UNDERSCORE})
set(PREFIX ${CMAKE_BINARY_DIR}/fakeroot)

if(MINGW)
  set(PLATFORM_OPTIONS target-os=windows threadapi=win32) # used to need this: threadapi=win32
else()
  set(PLATFORM_OPTIONS )
endif()

# We only need ZLIB and BZIP2 if we want to build the iostreams for zlib and bzip2.
set(COMMON_ARGS --toolset=gcc-${COMPILER_VERSION} --layout=system -sNO_ZLIB=1 -sNO_BZIP2=1 variant=release threading=multi link=shared runtime-link=shared --prefix=${PREFIX} $ENV{ROS_PARALLEL_JOBS})

# --with-date_time --with-filesystem --with-graph --with-graph_parallel
# --with-iostreams --with-math --with-mpi --with-program_options --with-python 
# --with-regex --with-serialization --with-signals --with-system --with-test
# --with-thread --with-wave
# Leaving out a few here that aren't usually necessary and some that are causing
# me cross compile breakages, e.g. wave
set(WITH_LIBS
     --with-date_time --with-filesystem 
     --with-iostreams --with-program_options
     --with-regex --with-signals --with-system
     --with-thread
     )

#set(BJAM_COMMAND ./bjam ${PACKAGE_OPTIONS} ${COMMON_ARGS} ${WITH_LIBS} install)
#set(BJAM_COMMAND ./bjam ${PLATFORM_OPTIONS} ${COMMON_ARGS} ${WITH_LIBS} install)
#set(BJAM_COMMAND ./bjam toolset=gcc-${COMPILER_VERSION} target-os=windows variant=release threading=multi threadapi=win32    link=shared runtime-link=shared --prefix=/opt/ros/stacks/eros/packages/rosdeps/core/eros_boost/build/fakeroot -j 2 --with-thread -sNO_BZIP2=1 -sNO_ZLIB=1 --layout=tagged install)
set(BJAM_COMMAND ./bjam ${PLATFORM_OPTIONS} ${COMMON_ARGS} ${WITH_LIBS} install)

if(CMAKE_CROSSCOMPILING)
  set(PACKAGE_NAME libboost-${SOURCE_VERSION}-${TOOLCHAIN_TUPLE}-dev)
else()
  set(PACKAGE_NAME libboost-${SOURCE_VERSION}-dev)
endif()

set(COMPILED_FLAG ${CMAKE_BINARY_DIR}/compiled)
set(LINKED_FLAG ${CMAKE_BINARY_DIR}/linked)
set(PATCHED_FLAG ${CMAKE_BINARY_DIR}/patched)

##############################################################################
# DOWNLOADING/GENERATING THE SOURCES
##############################################################################

eros_download(${SOURCE_TARBALL_URL} ${CMAKE_BINARY_DIR}/${SOURCE_TARBALL})
eros_extract_tarball(${CMAKE_BINARY_DIR}/${SOURCE_TARBALL} ${CMAKE_BINARY_DIR})

# Patch details for bjam to build
add_custom_command(OUTPUT ${PATCHED_FLAG}
    COMMAND echo "using gcc : ${COMPILER_VERSION} : ${CMAKE_CXX_COMPILER} : <compileflags>${ROS_COMPILE_FLAGS} ; " > ${SOURCE_DIR}/tools/build/v2/user-config.jam
    COMMAND touch ${PATCHED_FLAG}
    DEPENDS ${CMAKE_BINARY_DIR}/extracted
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Patching ${SOURCE_DIR}."
    VERBATIM
    )

##############################################################################
# Build
##############################################################################

add_custom_command(OUTPUT ${COMPILED_FLAG}
    COMMAND ./bootstrap.sh --without-icu
    COMMAND echo ${BJAM_COMMAND}
    COMMAND ${BJAM_COMMAND}
    COMMAND ${LN_COMMAND} 
    COMMAND touch ${COMPILED_FLAG}
    DEPENDS ${PATCHED_FLAG}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${SOURCE_DIR}
    COMMENT "Compiling ${SOURCE_NAME}."
    VERBATIM
    )

if(MINGW)
    set(LN_COMMAND ln -sf boost_thread_win32.dll ./boost_thread.dll) 
else()
    set(LN_COMMAND echo "Symbolic links automatically made.")
endif()

add_custom_command(OUTPUT ${LINKED_FLAG}
    COMMAND ${LN_COMMAND} 
    COMMAND touch ${LINKED_FLAG}
    DEPENDS ${COMPILED_FLAG}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/fakeroot/lib
    COMMENT "Symbolically linking ${SOURCE_NAME}."
    VERBATIM
    )


# By adding ALL here, it automatically gets built before the packaging command.
add_custom_target(
        source ALL
        DEPENDS ${LINKED_FLAG}
        COMMENT "This is ${PACKAGE_NAME}."
        )

##############################################################################
# Tests
##############################################################################

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
add_subdirectory(src)

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
        

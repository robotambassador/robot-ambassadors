#############################################################
# Ros specific settings
#############################################################
# Set the build type.  Options are:
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  RelWithAsserts : w/o debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
if(NOT DEFINED ROS_BUILD_TYPE)
  set(ROS_BUILD_TYPE RelWithDebInfo)
endif(NOT DEFINED ROS_BUILD_TYPE)

# Build static-only executables (e.g., for copying over to another
# machine)? true or false
if(NOT DEFINED ROS_BUILD_STATIC_EXES)
  set(ROS_BUILD_STATIC_EXES false)
endif(NOT DEFINED ROS_BUILD_STATIC_EXES)

# Build shared libs? true or false
if(NOT DEFINED ROS_BUILD_SHARED_LIBS)
  set(ROS_BUILD_SHARED_LIBS true)
endif(NOT DEFINED ROS_BUILD_SHARED_LIBS)

# Build static libs? true or false
if(NOT DEFINED ROS_BUILD_STATIC_LIBS)
  set(ROS_BUILD_STATIC_LIBS false)
endif(NOT DEFINED ROS_BUILD_STATIC_LIBS)

# Default compile flags for all source files
include(CheckCXXCompilerFlag)
if(NOT DEFINED ROS_COMPILE_FLAGS)
  set(ROS_COMPILE_FLAGS "-W -Wall -Wno-unused-parameter -fno-strict-aliasing")
  # Old versions of gcc need -pthread to enable threading, #2095.  
  # Also, some linkers, e.g., goLD, require -pthread (or another way to
  # generate -lpthread).
  # CYGWIN gcc has their -pthread disabled
  if(UNIX AND NOT CYGWIN)
    set(ROS_COMPILE_FLAGS "${ROS_COMPILE_FLAGS} -pthread")
  endif(UNIX AND NOT CYGWIN)
endif(NOT DEFINED ROS_COMPILE_FLAGS)

if(DEFINED PLATFORM_COMPILE_FLAGS)
  set(ROS_COMPILE_FLAGS "${ROS_COMPILE_FLAGS} ${PLATFORM_COMPILE_FLAGS}")
endif()

if(DEFINED TOOLCHAIN_COMPILE_FLAGS)
  set(ROS_COMPILE_FLAGS "${ROS_COMPILE_FLAGS} ${TOOLCHAIN_COMPILE_FLAGS}")
endif()

# Default link flags for all executables and libraries
if(NOT DEFINED ROS_LINK_FLAGS)
  set(ROS_LINK_FLAGS "")
  # Old versions of gcc need -pthread to enable threading, #2095.  
  # Also, some linkers, e.g., goLD, require -pthread (or another way to
  # generate -lpthread).
  # CYGWIN gcc has their -pthread disabled
  if(UNIX AND NOT CYGWIN)
    set(ROS_LINK_FLAGS "${ROS_LINK_FLAGS} -pthread")
  endif(UNIX AND NOT CYGWIN)
endif(NOT DEFINED ROS_LINK_FLAGS)

if(DEFINED PLATFORM_LINK_FLAGS)
  set(ROS_LINK_FLAGS "${ROS_LINK_FLAGS} ${PLATFORM_LINK_FLAGS}")
endif()

if(DEFINED TOOLCHAIN_LINK_FLAGS)
  set(ROS_LINK_FLAGS "${ROS_LINK_FLAGS} ${TOOLCHAIN_LINK_FLAGS}")
endif()

# Default libraries to link against for all executables and libraries
if(NOT DEFINED ROS_LINK_LIBS)
  set(ROS_LINK_LIBS "")
endif(NOT DEFINED ROS_LINK_LIBS)

if(DEFINED PLATFORM_LINK_LIBS)
  set(ROS_LINK_LIBS "${ROS_LINK_LIBS} ${PLATFORM_LINK_LIBS}")
endif()

#############################################################
# CMake Settings
#############################################################

set(CMAKE_INSTALL_PREFIX /usr/local/ CACHE PATH "Install location" FORCE)

# Hide from cache's front page
MARK_AS_ADVANCED(PLATFORM_NAME PLATFORM_FAMILY PLATFORM_COMPILE_FLAGS PLATFORM_LINK_FLAGS PLATFORM_LINK_LIBS)


###############################################################################
# Family : arm
# Platform : arm1176jzf-s
###############################################################################

# Some useful custom variables that uniquely define this platform module
set(PLATFORM_FAMILY "arm" CACHE STRING "Platform family, usually referring to intel/arm etc.")
set(PLATFORM_NAME "arm1176jzf-s" CACHE STRING "Platform name, usually referring to the cpu architecture.")

# Flags
set(PLATFORM_COMPILE_FLAGS "-march=armv6 -mtune=arm1176jzf-s -pipe -mfloat-abi=softfp -mfpu=vfp" CACHE STRING "Compile flags specific to this platform.")
set(PLATFORM_LINK_FLAGS "" CACHE STRING "Link flags specific to this platform.")

# Now the cmake variables
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} ${PLATFORM_COMPILE_FLAGS} CACHE STRING "Compile flags for c programs." FORCE) 
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${PLATFORM_COMPILE_FLAGS} CACHE STRING "Compile flags for c++ programs" FORCE)

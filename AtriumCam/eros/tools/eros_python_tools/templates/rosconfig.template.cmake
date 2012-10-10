###############################################################################
# Family : ${platform_family}
# Platform : ${platform_name}
###############################################################################

# Some useful custom variables that uniquely define this platform module
set(PLATFORM_FAMILY "${platform_family}" CACHE STRING "Platform family, usually referring to intel/arm etc.")
set(PLATFORM_NAME "${platform_name}" CACHE STRING "Platform name, usually referring to the cpu architecture.")

# Flags
set(PLATFORM_COMPILE_FLAGS "${platform_compile_flags}" CACHE STRING "Compile flags specific to this platform.")
set(PLATFORM_LINK_FLAGS "${platform_link_flags}" CACHE STRING "Link flags specific to this platform.")


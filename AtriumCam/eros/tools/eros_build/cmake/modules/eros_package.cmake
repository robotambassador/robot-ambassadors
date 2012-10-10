###############################################################################
# Version
###############################################################################

# Set the eros version.
macro(eros_version)
    set(PROJECT_VERSION "0.20.0")
    set(PROJECT_VERSION_MAJOR "0")
    set(PROJECT_VERSION_MINOR "20")
    set(PROJECT_VERSION_PATCH "0")
endmacro()

###############################################################################
# Init for a generic eros 3rd party build (aka rosbuild_init())
###############################################################################
# This must be called after rosbuild_init has been called - i.e. usually
#
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# rosbuild_init()
# rosbuild_include(eros_build eclbuild.cmake)
#

# This is the init used for building 3rd party packages in eros_cross.
macro(eros_3rd_party_init)
    rosbuild_find_ros_package(eros_build)
    set(CMAKE_MODULE_PATH ${eros_build_PACKAGE_PATH}/cmake/modules)
    include(eros_build_utilities)
    eros_version()
    eros_add_uninstall_target()
endmacro()

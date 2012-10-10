Original build was building a gtest the hard way with rosbuild_add_executable
and rosbuild_add_gtest_build_flags. As we don't cross-compile gtest, we don't
want to be building a gtest when running make. 

Moved this to the proper rosbuild_add_gtest api.

Submitted upstream, this is ticket https://code.ros.org/trac/ros/ticket/2823.

Two patches here.

===== private.cmake =====

This one is a critical fix in a find_file argument that forces find_file with
NO_CMAKE_FIND_ROOT_PATH.

Submitted upstream, this is ticket https://code.ros.org/trac/ros/attachment/ticket/2994/

===== rostoolchain.cmake =====

This just removes a rather noisy comment made by cmake that informs the user
that a toolchain has been selected. Unfortunately it is really spammy when 
cross-compiling and not really useful anyway (you can see you're cross-compiling
by viewing the cmake output for compiler and such pretty easily anyway).

Not yet committed upstream (not urgent).
###############################################################################
# Tests (not really gtests, just using the test framework)
###############################################################################

rosbuild_add_gtest(test_shared_pointer shared_pointer.cpp)
target_link_libraries(test_shared_pointer -L${CMAKE_BINARY_DIR}/fakeroot/lib -lboost_thread)

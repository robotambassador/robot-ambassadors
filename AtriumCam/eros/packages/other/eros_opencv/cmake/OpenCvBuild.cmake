
##############################################################################
# Svn
##############################################################################

set(SVN_URL https://code.ros.org/svn/opencv/tags/2.1/opencv)
set(SVN_DIR ${CMAKE_BINARY_DIR}/opencv-svn)
#set(SVN_REV 1978) # This is the tagged release number 
set(PATCH_FILE ${PROJECT_SOURCE_DIR}/build/opencv-svn/src/highgui/cvcap_dshow.cpp)
set(SRC_FILE ${PROJECT_SOURCE_DIR}/cvcap_dshow.cpp)

##############################################################################
# Checkout
##############################################################################

list(APPEND SVN_COMMAND svn export ${SVN_URL} ${SVN_DIR})
add_custom_command(OUTPUT ${SVN_DIR}
	COMMAND pwd
        COMMAND ${SVN_COMMAND}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Checking out ${SVN_URL} -> ${SVN_DIR}."
	COMMAND pwd
	COMMAND cp ${SRC_FILE} ${PATCH_FILE}
	COMMAND pwd
        )

##############################################################################
# Opencv Configuration
##############################################################################

# Check for sse's when the next ros release comes out.
set(OPENCV_BUILD_DIR ${SVN_DIR}/build)
set(OPENCV_INSTALL_DIR ${CMAKE_BINARY_DIR}/fakeroot)

if(CMAKE_CROSSCOMPILING)
    # Default everything to the worst common denominator - horrible assumption!
    set(OPENCV_SSE_FLAGS -DENABLE_SSSE3=OFF -DENABLE_SSE3=OFF -DENABLE_SSE2=OFF -DENABLE_SSE=OFF)
else()
    rosbuild_check_for_sse()
    set(OPENCV_SSE_FLAGS)
    if(HAS_SSE3_EXTENSIONS)
        set(OPENCV_SSE_FLAGS -DENABLE_SSSE3=OFF -DENABLE_SSE3=ON -DENABLE_SSE2=ON -DENABLE_SSE=ON)
    elseif(HAS_SSE2_EXTENSIONS)
        set(OPENCV_SSE_FLAGS -DENABLE_SSSE3=OFF -DENABLE_SSE3=OFF -DENABLE_SSE2=ON -DENABLE_SSE=ON)
    elseif(HAS_SSE_EXTENSIONS)
        set(OPENCV_SSE_FLAGS -DENABLE_SSSE3=OFF -DENABLE_SSE3=OFF -DENABLE_SSE2=OFF -DENABLE_SSE=ON)
    endif()
endif()
# ROS_BUILD_TYPE - defined above, default is RelWithDebInfo
rosbuild_find_ros_package(rosbuild)
set(ROS_TOOLCHAIN ${rosbuild_PACKAGE_PATH}/rostoolchain.cmake)
set(CMAKE_ARGS
		-DCMAKE_BUILD_TYPE=${ROS_BUILD_TYPE}
        -DCMAKE_INSTALL_PREFIX=${OPENCV_INSTALL_DIR}
        -DBUILD_TESTS=OFF
        -DBUILD_NEW_PYTHON_SUPPORT=OFF
        -DBUILD_SWIG_PYTHON_SUPPORT=OFF
		-DWITH_FFMPEG=OFF
		-DWITH_GSTREAMER=OFF
		-DWITH_GTK=OFF
		-DWITH_JASPER=OFF
		-DWITH_JPEG=ON
		-DWITH_PNG=ON
		-DWITH_PVAPI=OFF
		-DWITH_TBB=OFF
		-DWITH_TIFF=OFF
		-DWITH_UNICAP=OFF
		-DWITH_V4L=OFF
		-DWITH_XINE=OFF
#		-DWITH_1394=OFF
		-DCMAKE_TOOLCHAIN_FILE=${ROS_TOOLCHAIN}
		-DOPENCV_EXTRA_C_FLAGS=${ROS_COMPILE_FLAGS}
		${OPENCV_SSE_FLAGS}
)

# file(MAKE_DIRECTORY ${OPENCV_BUILD_DIR}) # Can't come first, because it breaks DEPENDS in ycssvn()
# so we do this below
add_custom_command(
    OUTPUT ${OPENCV_BUILD_DIR}
    COMMAND mkdir -p ${OPENCV_BUILD_DIR}
    WORKING_DIRECTORY ${SVN_DIR}
    DEPENDS ${SVN_DIR}
    )

add_custom_command(
    OUTPUT ${OPENCV_BUILD_DIR}/CMakeCache.txt
    COMMAND cmake ${CMAKE_ARGS} ..
    WORKING_DIRECTORY ${OPENCV_BUILD_DIR}
    DEPENDS ${OPENCV_BUILD_DIR}
    COMMENT "Configured opencv."
    )

##############################################################################
# Opencv Configuration
##############################################################################

add_custom_command(
    OUTPUT ${OPENCV_INSTALL_DIR}/lib/libcv.so
    COMMAND make $ENV{ROS_PARALLEL_JOBS}
    COMMAND make install
    WORKING_DIRECTORY ${OPENCV_BUILD_DIR}
    DEPENDS ${OPENCV_BUILD_DIR}/CMakeCache.txt
    COMMENT "Compiled opencv."
    )
set(BUILD_FIXED_FLAG ${CMAKE_SOURCE_DIR}/build/fixed)
add_custom_command(
    OUTPUT  ${BUILD_FIXED_FLAG}
    COMMAND ./change_install_names_for_mac
    COMMAND touch ${BUILD_FIXED_FLAG}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    DEPENDS ${OPENCV_INSTALL_DIR}/lib/libcv.so
    COMMENT "Fixed libraries (for mac only)."
    )

add_custom_target(
    opencv_build ALL
    DEPENDS ${BUILD_FIXED_FLAG}
    COMMENT "Opencv done."
    )

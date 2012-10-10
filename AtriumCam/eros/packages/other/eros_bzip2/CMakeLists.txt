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

set(SOURCE_NAME "bzip2-1.0.5")
set(SOURCE_TARBALL "${SOURCE_NAME}.tar.gz")
set(SOURCE_TARBALL_URL "http://www.bzip.org/1.0.5/${SOURCE_TARBALL}")
set(SOURCE_DIR ${CMAKE_BINARY_DIR}/${SOURCE_NAME})
set(PREFIX ${CMAKE_BINARY_DIR}/fakeroot)
set(COMMON_VARS CC=${CMAKE_C_COMPILER} RANLIB=${CMAKE_RANLIB} AR=${CMAKE_AR} PREFIX=${PREFIX})

##############################################################################
# Download
##############################################################################

eros_download(${SOURCE_TARBALL_URL} ${CMAKE_BINARY_DIR}/${SOURCE_TARBALL})
eros_extract_tarball(${CMAKE_BINARY_DIR}/${SOURCE_TARBALL} ${CMAKE_BINARY_DIR})

##############################################################################
# Build
##############################################################################

set(COMPILED_FLAG ${CMAKE_BINARY_DIR}/compiled)
set(FAKE_INSTALL_FLAG ${CMAKE_BINARY_DIR}/fake_install_flag)

add_custom_command(OUTPUT ${COMPILED_FLAG}
    COMMAND make -f Makefile-libbz2_so ${COMMON_VARS} 
	COMMAND echo "make install ${COMMON_VARS}"
    COMMAND make install ${COMMON_VARS}
    COMMAND touch ${COMPILED_FLAG}
    DEPENDS ${CMAKE_BINARY_DIR}/extracted
    WORKING_DIRECTORY ${SOURCE_DIR}
    COMMENT "Compiling ${SOURCE_DIR}."
    VERBATIM
    )

add_custom_command(OUTPUT ${FAKE_INSTALL_FLAG}
    COMMAND mkdir -p ${PREFIX}/lib
	COMMAND mkdir -p ${PREFIX}/include
	COMMAND cp -a libbz*so* ${PREFIX}/lib
	COMMAND cp *.h ${PREFIX}/include
    COMMAND touch ${FAKE_INSTALL_FLAG}
    DEPENDS ${COMPILED_FLAG}
    WORKING_DIRECTORY ${SOURCE_DIR}
    COMMENT "Gathering bzip2 headers, libs and binaries."
	#VERBATIM
    )


# By adding ALL here, it automatically gets built before the packaging command.
add_custom_target(
        source ALL
        DEPENDS ${FAKE_INSTALL_FLAG}
        COMMENT "This is ${PROJECT_NAME}."
        )

##############################################################################
# Install
##############################################################################

install(DIRECTORY ${PREFIX}/bin/
        DESTINATION bin
        USE_SOURCE_PERMISSIONS
        )
install(DIRECTORY ${PREFIX}/include/
        DESTINATION include
        USE_SOURCE_PERMISSIONS
        )
install(DIRECTORY ${PREFIX}/lib/
        DESTINATION lib
        USE_SOURCE_PERMISSIONS
        )


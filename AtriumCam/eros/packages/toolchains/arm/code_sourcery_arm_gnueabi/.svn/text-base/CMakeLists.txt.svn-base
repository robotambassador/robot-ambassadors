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
# Verbosity
##############################################################################

# Without this, we cant see the stdout from python's makes
set(CMAKE_VERBOSE_MAKEFILE TRUE)

##############################################################################
# Variables
##############################################################################

#set(VERSION_NAME arm-2010q1)
#set(BINARY_TARBALL arm-2010q1-202-arm-none-linux-gnueabi-i686-pc-linux-gnu.tar.bz2)
#set(BINARY_TARBALL_URL http://www.codesourcery.com/public/gnu_toolchain/arm-none-linux-gnueabi/${BINARY_TARBALL})
#set(SOURCE_TARBALL arm-2010q1-202-arm-none-linux-gnueabi.src.tar.bz2)
#set(SOURCE_TARBALL_URL http://www.codesourcery.com/public/gnu_toolchain/arm-none-linux-gnueabi/${SOURCE_TARBALL})
set(VERSION_NAME arm-2010.09)
set(BINARY_TARBALL ${VERSION_NAME}-50-arm-none-linux-gnueabi-i686-pc-linux-gnu.tar.bz2)
set(BINARY_TARBALL_URL http://www.codesourcery.com/sgpp/lite/arm/portal/package7851/public/arm-none-linux-gnueabi/${BINARY_TARBALL})
set(SOURCE_TARBALL ${VERSION_NAME}-50-arm-none-linux-gnueabi.src.tar.bz2)
set(SOURCE_TARBALL_URL http://www.codesourcery.com/sgpp/lite/arm/portal/package7850/public/arm-none-linux-gnueabi/${SOURCE_TARBALL})

##############################################################################
# Download
##############################################################################

if( CMAKE_SIZEOF_VOID_P EQUAL 4 ) # 32 bit
  eros_download(${BINARY_TARBALL_URL} ${CMAKE_BINARY_DIR}/${BINARY_TARBALL})
  eros_extract_bzip2(${CMAKE_BINARY_DIR}/${BINARY_TARBALL} ${CMAKE_BINARY_DIR})
else()
  message(FATAL_ERROR "\n64 bit install is not yet supported - download/compile from source:\n  ${SOURCE_TARBALL_URL}")
endif()

##############################################################################
# Target
##############################################################################

# Forcing here, not ideal, but easiest solution to get it into the preferred
# location without human error.
set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "Install prefix, used if a relative path is given." FORCE)

# By adding ALL here, it automatically gets built before the packaging command.
add_custom_target(
        source ALL
        DEPENDS ${CMAKE_BINARY_DIR}/extracted
        COMMENT "Run 'make install' -> /usr. To change location, edit CMAKE_INSTALL_PREFIX in the CMakeLists.txt."
        )

##############################################################################
# Install
##############################################################################

install(DIRECTORY ${CMAKE_BINARY_DIR}/${VERSION_NAME}/lib/
        DESTINATION lib
        USE_SOURCE_PERMISSIONS
        )

install(DIRECTORY ${CMAKE_BINARY_DIR}/${VERSION_NAME}/share/
        DESTINATION share
        USE_SOURCE_PERMISSIONS
        )

install(DIRECTORY ${CMAKE_BINARY_DIR}/${VERSION_NAME}/libexec/
        DESTINATION libexec
        USE_SOURCE_PERMISSIONS
        )

install(DIRECTORY ${CMAKE_BINARY_DIR}/${VERSION_NAME}/bin/
        DESTINATION bin
        USE_SOURCE_PERMISSIONS
        )

install(DIRECTORY ${CMAKE_BINARY_DIR}/${VERSION_NAME}/arm-none-linux-gnueabi/
        DESTINATION arm-none-linux-gnueabi
        USE_SOURCE_PERMISSIONS
        )
        

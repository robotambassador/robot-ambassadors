###############################################################################
# Build utilities
###############################################################################

###############################
# Compiler Version
###############################
# Configures the variables:
# 
#  - COMPILER_VERSION
#
macro(eros_detect_compiler_version)
    if(${MSVC})
      set(COMPILER_VERSION ${MSVC})
    else()
      execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} --version
        OUTPUT_VARIABLE COMPILER_VERSION
        )
      string(REGEX REPLACE ".* ([0-9])\\.([0-9])\\.([0-9]).*" "\\1.\\2.\\3"
                         COMPILER_VERSION ${COMPILER_VERSION})
    endif()
endmacro()

###############################
# Download
###############################
# Adds a custom command for downloading a url to a 
# particular file.
#
# Depends : -
# Outputs : ${file}
#
# Note: ${file} can be 
# - relative in which case it is rel to ${CMAKE_BINARY_DIR}
# - absolute (e.g. ${CMAKE_BINARY_DIR}/dude.tar.gz
#
# Example:
#
#   URL=http://snorriheim.dnsdojo.com/tmp/${TARBALL}
#   TARBALL=dude.tar.gz
#   eros_download(${URL} ${TARBALL})
#
macro(eros_download url file)
    add_custom_command(OUTPUT ${file}
        COMMAND wget ${url} -O ${file}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Downloading ${url}->${file}."
        )
endmacro()

###############################
# Extract
###############################
# Adds a custom command for untar'ing a tarball in the specified dir.
#
# Depends : ${tarball}
# Outputs : ${dir}/extracted
# 
# Example:
# 
#   TARBALL=${CMAKE_BINARY_DIR}/dude.tar.gz
#   URL=http://snorriheim.dnsdojo.com/tmp/${TARBALL}
#   eros_download(${URL} ${TARBALL})
#   eros_extract_tarball(${TARBALL} ${CMAKE_BINARY_DIR}/fakeroot) 
#
macro(eros_extract_tarball tarball dir)
    add_custom_command(OUTPUT ${dir}/extracted
        COMMAND mkdir -p ${dir}
        COMMAND tar -xvzf ${tarball} -C ${dir}
        COMMAND touch ${dir}/extracted
        DEPENDS ${tarball}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Extracting ${tarball} -> ${dir}."
        VERBATIM
        )
endmacro()

# Similar to the untarball command, but for bzips.
#
macro(eros_extract_bzip2 bzip2 dir)
    add_custom_command(OUTPUT ${dir}/extracted
        COMMAND tar -xvjf ${bzip2} -C ${dir}
        COMMAND touch ${dir}/extracted
        DEPENDS ${bzip2}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Extracting ${bzip2} -> ${dir}."
        VERBATIM
        )
endmacro()


# Similar to the untarball command, but for zips.
#
macro(eros_extract_zip zip dir)
    add_custom_command(OUTPUT ${dir}/extracted
        COMMAND unzip ${zip} -d ${dir}
        COMMAND touch ${dir}/extracted
        DEPENDS ${zip}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Extracting ${zip} -> ${dir}."
        VERBATIM
        )
endmacro()

###############################
# Autotools Compile
###############################
# This adds a custom command for a typical autotools compile.
# Be sure to set up the configure command correctly before calling.
#
# Depends: ${depends} (input argument)
# Outputs: ${dir}/compiled
#
macro(eros_autotools_compile configure_command dir depends)
    add_custom_command(OUTPUT ${dir}/compiled
        COMMAND ${${configure_command}}
        COMMAND make $ENV{ROS_PARALLEL_JOBS}
        COMMAND make install
        COMMAND touch ${dir}/compiled
        DEPENDS ${depends}
        WORKING_DIRECTORY ${dir}
        COMMENT "Compiling ${dir}."
        VERBATIM
        )
endmacro()

###############################
# Autotools Compile Only
###############################
# This adds a custom command for a typical autotools compile.
# Be sure to set up the configure command correctly before calling.
#
# Depends: ${depends} (input argument)
# Outputs: ${dir}/compiled
#
macro(eros_autotools_compile_only configure_command dir depends)
    add_custom_command(OUTPUT ${dir}/compiled
        COMMAND ${${configure_command}}
        COMMAND make $ENV{ROS_PARALLEL_JOBS}
        COMMAND touch ${dir}/compiled
        DEPENDS ${depends}
        WORKING_DIRECTORY ${dir}
        COMMENT "Compiling ${dir}."
        VERBATIM
        )
endmacro()

###############################
# Uninstall
###############################
# Creates an uninstall target.
# 
# To do this, it needs to find the uninstall template and prep it.
# In ros, we can grab it from eros_build, otherwise we use pkgconfig
# to find it from an already installed ecl.
macro(eros_add_uninstall_target)

    # Find the template
    rosbuild_find_ros_package(eros_build)
    set(EROS_CMAKE_TEMPLATES_PATH ${eros_build_PACKAGE_PATH}/cmake/templates)

    # Prep it
    configure_file(
        "${EROS_CMAKE_TEMPLATES_PATH}/uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake"
        IMMEDIATE @ONLY)

    # Add the target
    add_custom_target(uninstall
        "${CMAKE_COMMAND}" -P
        "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake")
endmacro()

###############################
# Qt4
###############################
#
# On mingw_cross, system lib deps are in prl files which FindQt4 does not check, 
# this can lead to some shortages in the link dependencies especially when 
# statically building. So...if there is prl files to tell us about this, use them.
#
macro(qt_link_flags_from_prl component)
    string(TOUPPER ${component} COMPONENT_UPPER) 
	set(FILE_PRL "NOTFOUND")
 	find_file(FILE_PRL ${component}.prl ${QT_LIBRARY_DIR})
 	if( FILE_PRL )
 		file(READ ${FILE_PRL} PRL_CONTENTS) # Read .prl file
 		string(REGEX MATCH "QMAKE_PRL_LIBS.+" PRL_LINE ${PRL_CONTENTS}) # Extract the line with linking flags
 		string(REGEX REPLACE "-l" "" PRL_LINE ${PRL_LINE}) # Remove link flag symbols
 		string(REGEX REPLACE "QMAKE_PRL_LIBS" "" PRL_LINE ${PRL_LINE}) # Remove non links
 		string(REGEX REPLACE "=" "" PRL_LINE ${PRL_LINE}) # Remove non links
 		string(REGEX MATCHALL "[^ \t\n]+" PRL_LINK_FLAGS ${PRL_LINE}) # Split the line by whitespaces
 		# Don't double up in QT_LIBRARIES
 		if(PRL_LINK_FLAGS)
     		list(REMOVE_ITEM QT_LIBRARIES ${PRL_LINK_FLAGS})
     		list(APPEND QT_LIBRARIES ${PRL_LINK_FLAGS})
     		if(QT_${COMPONENT_UPPER}_LIB_DEPENDENCIES)
         		list(REMOVE_ITEM QT_${COMPONENT_UPPER}_LIB_DEPENDENCIES ${PRL_LINK_FLAGS})
         	endif()
       		list(APPEND QT_${COMPONENT_UPPER}_LIB_DEPENDENCIES ${PRL_LINK_FLAGS})
     	endif()
 	endif( FILE_PRL )
endmacro()
#
# Is a wrapper around FindQt4.cmake.
# 
# Mostly this is needed to satisfy a static mingw cross compile of qt, (mingw_cross)
# as the cmake FindQt4 handles this poorly. Call with the components you want to 
# compile for, e.g.
# 
#     eros_prepare_qt4(QtGui QtCore)
#
# What this macro does:
#
# - calls the underlying FindQt4.cmake module.
# - if compiling statically
#   - turns off -DQT_DLL if it's accidentally left on.
#   - checks to see if there are .prl's which can fill out the lib dependencies.
macro(eros_prepare_qt4)
    if ( ${ARGC} GREATER 0 ) 
        #foreach(arg ${ARGV})
        #  set(QT_COMPONENTS_ "${QT_COMPONENTS_} ${arg}")
        #endforeach()
        #string(STRIP ${QT_COMPONENTS_} QT_COMPONENTS_)
        #message("ARGV.........................${ARGV}")
        #message("QT_COMPONENTS................${QT_COMPONENTS_}")
        #message("QT_DEFINITIONS...............${QT_DEFINITIONS}")
        #find_package(Qt4 COMPONENTS ${QT_COMPONENTS_}) 
        #find_package(Qt4 COMPONENTS QtCore QtGui QtXml) 
        find_package(Qt4 COMPONENTS ${ARGV}) 
    else()
        find_package(Qt4)
    endif()
    
    # This is needed on my ubuntu (cmake 2.8.0), but not on gentoo (cmake 2.8.1)
    # Probably later this can be dropped as it is done in 2.8.1+
    if(QT_IS_STATIC) 
        list(REMOVE_ITEM QT_DEFINITIONS -DQT_DLL)
    endif()
    
    include(${QT_USE_FILE})
    include_directories(${CMAKE_CURRENT_BINARY_DIR}) # Needed to pick up ui files in build dir
    
    if(QT_IS_STATIC) 
        if(QT_USE_QAXCONTAINER) 
            qt_link_flags_from_prl(QAxContainer)
        endif()
        if(QT_USE_QAXSERVER) 
            qt_link_flags_from_prl(QAxServer)
        endif()
        if(QT_USE_QT3SUPPORT) 
            qt_link_flags_from_prl(Qt3Support)
        endif()
        if(QT_USE_QTCORE) 
            qt_link_flags_from_prl(QtCore)
        endif()
        if(QT_USE_QTGUI) 
            qt_link_flags_from_prl(QtGui)
        endif()
        if(QT_USE_QTMAIN) 
            qt_link_flags_from_prl(qtmain)
        endif()
        if(QT_USE_QTMULTIMEDIA) 
            qt_link_flags_from_prl(QtMultimedia)
        endif()
        if(QT_USE_QTNETWORK) 
            qt_link_flags_from_prl(QtNetwork)
        endif()
        if(QT_USE_QTOPENGL) 
            qt_link_flags_from_prl(QtOpenGL)
        endif()
        if(QT_USE_QTSCRIPT) 
            qt_link_flags_from_prl(QtScript)
        endif()
        if(QT_USE_QTSCRIPTTOOLS) 
            qt_link_flags_from_prl(QtScriptTools)
        endif()
        if(QT_USE_QTSQL) 
            qt_link_flags_from_prl(QtSql)
        endif()
        if(QT_USE_QTSVG) 
            qt_link_flags_from_prl(QtSvg)
        endif()
        if(QT_USE_QTTEST) 
            qt_link_flags_from_prl(QtTest)
        endif()
        if(QT_USE_QTWEBKIT) 
            qt_link_flags_from_prl(QtWebKit)
        endif()
        if(QT_USE_QTXMLPATTERNS) 
            qt_link_flags_from_prl(QtXmlPatterns)
        endif()
        if(QT_USE_QTXML) 
            qt_link_flags_from_prl(QtXml)
        endif()
    endif()
endmacro()

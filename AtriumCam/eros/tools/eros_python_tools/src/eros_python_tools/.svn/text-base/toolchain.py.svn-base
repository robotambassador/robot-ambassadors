'''
Created on 17/08/2010

This manages the installation of a toolchain cmake module from the toolchain library (eros_toolchains) 
in ${ROS_ROOT}/rostoolchain.cmake. It also ensures an appropriate platform is set and configured. 

@author: Daniel Stonier
'''
###############################################################################
# Imports
###############################################################################

import roslib
import os
import sys
import core
import shutil
import re
# eros_python_tools modules
import prefix
import platform

###############################################################################
# Interface [Toolchain]
###############################################################################

class Toolchain:
    '''
    Represents data about a toolchain:
      pathname : full pathname to the toolchain
      name : typically the toolchain tuple, e.g. i686-pc-linux-gnu
      group : valid groups are eros||user
      family : which family the toolchain belongs, e.g. crossdev||ubuntu||...
      current : true if it is the currently configured ros toolchain
    '''

    def __init__(self, toolchain_dir, toolchain_pathname, toolchain_id):
        # e.g. 
        #  toolchain_dir = /home/snorri/.ros/eros/toolchains
        #  toolchain_pathname = /home/snorri/.ros/eros/toolchains/crossdev/i686-pc-linux-gnu.cmake
        self.pathname = toolchain_pathname
        # Could check here, but if calling from toolchain_list, will always be arg 1 we want.
        tail = self.pathname.split(toolchain_dir)[1] # e.g. /crossdev/i686-pc-linux-gnu.cmake 
        cmake_name = os.path.basename(tail) # e.g. i686-pc-linux-gnu.cmake
        self.name = os.path.splitext(cmake_name)[0] # e.g. i686-pc-linux-gnu
        if ( self.pathname.find(eros_toolchain_dir()) != -1 ):
            self.group = "eros"
        else:
            self.group = "user"
        self.family = os.path.dirname(toolchain_pathname).split(toolchain_dir)[1] # e.g. /crossdev
        self.family = os.path.split(self.family)[1] # remove dir separators e.g. crossdev
        if ( self.family == '' ):
            self.family = "unknown"
        toolchain_exists = os.path.exists(core.rostoolchain_cmake())
        self.current = False
        if ( toolchain_exists ) :
            name_string = "TOOLCHAIN_TUPLE \"" + self.name + "\""
            if name_string in open(core.rostoolchain_cmake()).read():
                family_string = "TOOLCHAIN_FAMILY \"" + self.family + "\""
                if family_string in open(core.rostoolchain_cmake()).read():
                    self.current = True
        self.id = toolchain_id
#        print "Pathname: " + self.pathname
#        print "Tuple " + self.name
#        print "Group: " + self.group
#        print "Family: " + self.family
#        if ( self.current ):
#            print "Current: True"
#        else:
#            print "Current: False"
#        print "Id: " + repr(self.id)

    def validate(self):
        """
        Attempt to validate the configuration of this toolchain. It checks for
        1) cross-compiler binaries in the PATH.
        2) the existence of the sysroot (though no guarantee it is the sysroot).
        3) a small program can be compiled.
        """
        if ( platform.system == 'Windows' ):
            toolchain_gcc = self.name + "-gcc.exe"
            toolchain_gpp = self.name + "-g++.exe"
        else:
            toolchain_gcc = self.name + "-gcc"
            toolchain_gpp = self.name + "-g++"
        toolchain_gcc_found = False 
        toolchain_gpp_found = False 
        path = os.environ['PATH']
        paths = path.split(os.pathsep)
        for dir in paths:
            if ( os.path.isdir(dir) ):
                pathname = os.path.join(dir,toolchain_gcc)
                if ( os.path.isfile(pathname) ):
                    toolchain_gcc_pathname = pathname
                    toolchain_gcc_found = True
                pathname = os.path.join(dir,toolchain_gpp)
                if ( os.path.isfile(pathname) ):
                    toolchain_gpp_pathname = pathname
                    toolchain_gpp_found = True
        if ( ( not toolchain_gcc_found ) or ( not toolchain_gpp_found ) ):
            print core.red_string("Cross-compiler binaries not found.")
            return 1
        print
        print "  Compilers validated: "
        print "    gcc : " + toolchain_gcc_pathname
        print "    gpp : " + toolchain_gpp_pathname
        
###############################################################################
# Methods
###############################################################################
    
def eros_toolchain_template():
    return os.path.join(roslib.packages.get_pkg_dir('eros_python_tools'),"templates","rostoolchain.template.cmake")
    
def eros_toolchain_dir():
    return os.path.join(roslib.packages.get_pkg_dir('eros_toolchains'),"library")

# global variable for the user toolchains dir, access via user_toolchain_dir()
_user_toolchains_dir = os.path.join(core.eros_home(),"toolchains")

def user_toolchain_dir(dir=None):
    global _user_toolchains_dir
    if ( dir != None ):
        if ( not os.path.exists(dir) ):
            raise Exception('Specified directory does not exist.')
        _user_toolchains_dir = dir
    return _user_toolchains_dir

# global variable for the toolchain list, access via toolchain_list()
toolchains = []

def toolchain_list():
    """
    Both provides access to and populates the toolchain list from both eros and user-defined
    toolchain libraries.
    """
    global toolchains
    if ( len(toolchains) != 0 ): 
        return toolchains
    # else populate from eros and user-defined libraries
    id_counter = 1
    for root, unused_dirs, files in os.walk(eros_toolchain_dir()):
        for name in files:
            if name.endswith('.cmake'):
                toolchains += [Toolchain(eros_toolchain_dir(),os.path.join(root,name), id_counter)]
                id_counter += 1
    for root, unused_dirs, files in os.walk(user_toolchain_dir()):
        for name in files:
            if name.endswith('.cmake'):
                toolchains += [Toolchain(user_toolchain_dir(),os.path.join(root,name), id_counter)]
                id_counter += 1
    return toolchains 
    
def list_eros_toolchains():
    toolchains = toolchain_list()
    print
    print core.bold_string("Eros toolchains:")
    print
    for toolchain in toolchains:
        if ( toolchain.group == 'eros' ):
            if ( toolchain.current ):
                print "  %d) %s%s%s%s" %(toolchain.id,toolchain.family,os.sep,toolchain.name,core.red_string("*"))
            else:
                print "  %d) %s%s%s" %(toolchain.id,toolchain.family,os.sep,toolchain.name)

def list_user_toolchains():
    toolchains = toolchain_list()
    print
    print core.bold_string("User toolchains:")
    print
    for toolchain in toolchains:
        if ( toolchain.group == 'user' ):
            if ( toolchain.current ):
                print "  %d) %s%s%s%s" %(toolchain.id,toolchain.family,os.sep,toolchain.name,core.red_string("*"))
            else:
                print "  %d) %s%s%s" %(toolchain.id,toolchain.family,os.sep,toolchain.name)

def list_toolchains():
    list_eros_toolchains()
    list_user_toolchains()
    print
    
def show_current_toolchain():
    '''
    Print the identity of the currently configured toolchain:
      - checks eros/user toolchain libraries for a match
      - if not eros/user toolchain, checks if toolchain configured, but unknown
      - otherwise prints none 
    '''
    pretext = core.bold_string("Current toolchain: ")
    toolchains = toolchain_list()
    found = False
    for toolchain in toolchains:
        if ( toolchain.current ):
            found = True
            current_toolchain = toolchain
    if ( found ):
        print pretext + current_toolchain.family + os.sep + current_toolchain.name
    else:
        if ( os.path.exists(core.rostoolchain_cmake()) ):
            print pretext + "unknown"
        else:
            print pretext + "none"

def select_toolchain():
    '''
    Interactively selects and sets an ros toolchain.
      - return true or false depending on success/failure.
    '''
    list_toolchains()
    toolchain_id_string = raw_input("Enter a toolchain id #: ")
    if ( not toolchain_id_string.isdigit() ):
        print core.red_string("Aborting, invalid id #.")
        return False
    toolchain_id = int(toolchain_id_string)
    toolchains = toolchain_list()
    if not toolchain_id <= len(toolchains):
        print core.red_string("Aborting, invalid id #.")
        return False
    found_toolchain = False
    for toolchain in toolchains:
        if ( toolchain.id == toolchain_id ):
            found_toolchain = True
            selected_toolchain = toolchain
            break
    if ( not found_toolchain ):
        print core.red_string("Aborting, toolchain not found.")
        return False
    else:
        shutil.copyfile(selected_toolchain.pathname,core.rostoolchain_cmake())
        print
        return True

def select_toolchain_by_name(id_string):
    '''
    Selects toolchain by name or family/name string.
      - return 1 if failure, 0 if success
    '''
    bits = os.path.split(id_string)
    #print bits
    if  len(bits) == 2 :
        family = bits[0] # if just name is given, this will be empty
        name = bits[1]
        found_toolchain = False
        toolchains = toolchain_list()
        if family == "": # try and just match name
            for toolchain in toolchains:
                if ( toolchain.name == name ):
                    if found_toolchain:
                        print
                        print core.red_string("Multiple matches, please provide a full id string.")
                        list_toolchains()
                        return 1
                    else:
                        found_toolchain = True
                        selected_toolchain = toolchain
        else: # try and match both family, name
            for toolchain in toolchains:
                if toolchain.family == family and toolchain.name == name:
                    found_toolchain = True
                    selected_toolchain = toolchain
                    break
        if ( not found_toolchain ):
            print
            print core.red_string("Aborting, toolchain not found.")
            list_toolchains()
            return 1
        else:
            shutil.copyfile(selected_toolchain.pathname,core.rostoolchain_cmake())
            return 0
    else:
        print
        print core.red_string("Aborting, toolchain not found.")
        list_toolchains()
        return 1

def select_toolchain_by_id(id):
    '''
    Selects toolchain by id.
      - return 1 if failure, 0 if success
    '''
    toolchains = toolchain_list()
    if ( int(id) > len(toolchains) ):
        print core.red_string("-- Aborting, # does not correspond to a toolchain.")
        list_toolchains()
        return 1
    else:
        selected_toolchain = toolchains[int(id)-1] # indexing starts at zero
        shutil.copyfile(selected_toolchain.pathname,core.rostoolchain_cmake())
        return 0

def delete_toolchain():
    '''
    Interactively deletes a user-defined toolchain.
      - return 1 if failure, 0 if success
    '''
    list_user_toolchains()
    print
    toolchain_id_string = raw_input("Enter a toolchain id #: ")
    if ( not toolchain_id_string.isdigit() ):
        print core.red_string("Aborting, invalid id #.")
        return 1
    toolchain_id = int(toolchain_id_string)
    toolchains = toolchain_list()
    if not toolchain_id <= len(toolchains):
        print core.red_string("Aborting, invalid id #.")
        return 1
    found_toolchain = False
    for toolchain in toolchains:
        if ( toolchain.id == toolchain_id ):
            if ( toolchain.group == 'user' ):
                found_toolchain = True
                selected_toolchain = toolchain
    if ( not found_toolchain ):
        print core.red_string("Aborting, invalid id #.") # probably passed an eros toolchain id
        return 1
    else:
        os.remove(selected_toolchain.pathname)
        return 0

def create_toolchain():
    '''
    Create a cross-compiler cmake configuration. Note: hardwired for gcc
    cross compiler configurations at this point in time - is there even a use
    case that is different right now?
    '''
    print
    print core.bold_string("  Creating a User-Defined Eros Toolchain")
    print
    print "This is an interactive assistant to help define a new eros style cmake toolchain."
    print "It will prompt you for a few custom strings and then save the configured toolchain"
    print "in ROS_HOME/eros/toolchains (~/.ros/eros/toolchains on linux platforms). It can then be"
    print "listed and selected in the same way as as a regular eros toolchain."
    print
    print core.bold_string("  Toolchain Family")
    print 
    print "  This is simply a convenience variable that helps sort toolchains in the eros"
    print "  and user-defined libraries. Common examples include: crossdev, ubuntu,"
    print "  openembedded, etc."
    print
    toolchain_family = raw_input('  Enter a string for the toolchain family [custom]: ')
    if ( toolchain_family == '' ):
        toolchain_family = 'custom'
    print
    print core.bold_string("  Toolchain Tuple")
    print 
    print "  This is essential so that cmake can find the gcc cross-compiler. The toolchain"
    print "  tuple should match the prefix to your toolchain's cross-compilers, e.g. if your"
    print "  cross-compiler is i686-pc-linux-gnu-gcc, then the tuple is i686-pc-linux-gnu."
    print "  Compilers need to be in your system's PATH."
    print
    toolchain_tuple = raw_input('  Enter a string for the toolchain tuple: ')
    print
    print core.bold_string("  Toolchain Sysroot")
    print 
    print "  This is the root directory from which system headers and libraries can be found."
    print "  e.g. if toolchain pthreads header is /usr/i686-pc-linux-gnu/usr/include/pthread.h"
    print "  then your sysroot would be /usr/i686-pc-linux-gnu."
    print 
    toolchain_sysroot_default = "/usr/" + toolchain_tuple
    toolchain_sysroot = raw_input("  Enter a string for the toolchain sysroot [" + toolchain_sysroot_default + "]: ")
    if ( toolchain_sysroot == ''):
        toolchain_sysroot = toolchain_sysroot_default
    print
    print core.bold_string("  Toolchain Install Prefix")
    print 
    print "  This is the where your headers and libraries will get installed."
    print "  e.g. if your toolchain sysroot is /usr/i686-pc-linux-gnu/ then typically"
    print "  your install prefix will be /usr/i686-pc-linux-gnu/usr"
    print 
    toolchain_install_prefix_default = toolchain_sysroot + "/usr/"
    toolchain_install_prefix = raw_input("  Enter a string for the toolchain install prefix [" + toolchain_install_prefix_default + "]: ")
    if ( toolchain_install_prefix == ''):
        toolchain_install_prefix = toolchain_install_prefix_default
    print
    
    toolchain_template = open(eros_toolchain_template()).read()
    toolchain_template = toolchain_template.replace('${toolchain_family}',toolchain_family)
    toolchain_template = toolchain_template.replace('${toolchain_tuple}',toolchain_tuple)
    toolchain_template = toolchain_template.replace('${toolchain_sysroot}',toolchain_sysroot)
    toolchain_template = toolchain_template.replace('${toolchain_install_prefix}',toolchain_install_prefix)
    #print toolchain_template
    user_defined_toolchain_pathname = os.path.join(user_toolchain_dir(),toolchain_family,toolchain_tuple+'.cmake')
    if ( os.path.exists(user_defined_toolchain_pathname) ):
        print core.red_string("  Aborting, this toolchain configuration already exists (use --delete to remove).")
        return 1
    if not os.path.exists( os.path.dirname(user_defined_toolchain_pathname) ): # Make sure the dir exists before we open for writing
        os.makedirs(os.path.dirname(user_defined_toolchain_pathname))
    f = open(user_defined_toolchain_pathname, 'w')
    f.write(toolchain_template)
    f.close()
    print core.bold_string("Toolchain Finalised")
    print
    print "-- Family: %s" %toolchain_family
    print "-- Tuple: %s" %toolchain_tuple
    print "-- Sysroot: %s" %toolchain_sysroot
    print "-- Install Prefix: %s" %toolchain_install_prefix
    print "-- File: %s" %user_defined_toolchain_pathname
    print
    
def check_platform():
    rosconfig_exists = os.path.exists(core.rosconfig_cmake())
    if rosconfig_exists:
        print "-- Found rosconfig.cmake."
        print "  -- Setting the install prefix to ${TOOLCHAIN_INSTALL_PREFIX}"
        prefix.set_install_prefix("${TOOLCHAIN_INSTALL_PREFIX}")
        print "  -- Confirm that it is compatible with the current toolchain."
    else:
        print "-- No rosconfig.cmake, generating a default (vanilla) configuration."
        platform.select_default()

def patch_ros():
    version = core.ros_version()
    patches_dir = os.path.join(roslib.packages.get_pkg_dir('eros_python_tools'),"patches",version)
    if ( version == 'cturtle' ):
        print "-- Applied various cross-compiling patches for cturtle."
        # rosccpp - cedric's patches.
        roscpp_dir = os.path.join(roslib.packages.get_pkg_dir('roscpp'),'src','libros')
        roscpp_init = os.path.join(patches_dir,"roscpp","init.cpp")
        roscpp_spinner = os.path.join(patches_dir,"roscpp","spinner.cpp")
        roscpp_poll_manager = os.path.join(patches_dir,"roscpp","poll_manager.cpp")
        roscpp_xmlrpc_manager = os.path.join(patches_dir,"roscpp","xmlrpc_manager.cpp")
        shutil.copyfile(roscpp_init,os.path.join(roscpp_dir,'init.cpp'))
        shutil.copyfile(roscpp_spinner,os.path.join(roscpp_dir,'spinner.cpp'))
        shutil.copyfile(roscpp_poll_manager,os.path.join(roscpp_dir,'poll_manager.cpp'))
        shutil.copyfile(roscpp_xmlrpc_manager,os.path.join(roscpp_dir,'xmlrpc_manager.cpp'))
        # genmsg_cpp
        genmsg_cpp_dir = roslib.packages.get_pkg_dir('genmsg_cpp')
        genmsg_cmakelists = os.path.join(patches_dir,"genmsg_cpp","CMakeLists.txt")
        genmsg_makefile = os.path.join(patches_dir,"genmsg_cpp","Makefile")
        shutil.copyfile(genmsg_cmakelists,os.path.join(genmsg_cpp_dir,'CMakeLists.txt'))
        shutil.copyfile(genmsg_makefile,os.path.join(genmsg_cpp_dir,'Makefile'))
        # message_filters
        message_filters_dir = roslib.packages.get_pkg_dir('message_filters')
        message_filters_cmakelists = os.path.join(patches_dir,"message_filters","test","CMakeLists.txt")
        shutil.copyfile(message_filters_cmakelists,os.path.join(message_filters_dir,"test","CMakeLists.txt"))
        # rosbuild
        rosbuild_dir = roslib.packages.get_pkg_dir('rosbuild')
        rostoolchain_cmake = os.path.join(patches_dir,"rosbuild","rostoolchain.cmake")
        private_cmake = os.path.join(patches_dir,"rosbuild","private.cmake")
        shutil.copyfile(rostoolchain_cmake,os.path.join(rosbuild_dir,'rostoolchain.cmake'))
        shutil.copyfile(private_cmake,os.path.join(rosbuild_dir,'private.cmake'))
        # rospack
        rospack_dir = roslib.packages.get_pkg_dir('rospack')
        rospack_cmakelists = os.path.join(patches_dir,"rospack","CMakeLists.txt")
        rospack_makefile = os.path.join(patches_dir,"rospack","Makefile")
        shutil.copyfile(rospack_cmakelists,os.path.join(rospack_dir,'CMakeLists.txt'))
        shutil.copyfile(rospack_makefile,os.path.join(rospack_dir,'Makefile'))
        # topic_tools
        topic_tools_dir = roslib.packages.get_pkg_dir('topic_tools')
        topic_tools_cmakelists = os.path.join(patches_dir,"topic_tools","CMakeLists.txt")
        shutil.copyfile(topic_tools_cmakelists,os.path.join(topic_tools_dir,'CMakeLists.txt'))
    elif ( version == 'diamondback' ):
        overlay_patches(version)
    else:
        print "-- Assuming you have unstable or trunk installed."
        print "-- Will use patches from diamondback which should be compatible."
        #overlay_patches('diamondback')

def overlay_patches(version):
    print "-- Applied various cross-compiling patches for " + version + "." 
    patches_dir = os.path.join(roslib.packages.get_pkg_dir('eros_python_tools'),"patches",version,"updates")
    ros_patches_dir = os.path.join(patches_dir,"ros")
    ros_comm_patches_dir = os.path.join(patches_dir,"ros_comm")
    common_msgs_patches_dir = os.path.join(patches_dir,"common_msgs")
    ros_dir = roslib.stacks.get_stack_dir('ros')
    ros_comm_dir = roslib.stacks.get_stack_dir('ros_comm')
    common_msgs_dir = roslib.stacks.get_stack_dir('common_msgs')
    copy_tree(ros_patches_dir,ros_dir)
    copy_tree(ros_comm_patches_dir,ros_comm_dir)
    copy_tree(common_msgs_patches_dir,common_msgs_dir)
    #shutil.copytree(ros_patches_dir,ros_dir,ignore=shutil.ignore_patterns(IGNORE_PATTERNS))

def copy_tree(src_dir, dest_dir):
    if ( not os.path.exists(dest_dir) ) : 
        os.makedirs(dest_dir)
    for dir, unused_subdirs, files in os.walk(src_dir):
        if re.search(".svn",dir):
            continue
        new_dir = dir.replace(src_dir,dest_dir)
        for file in files:
            #print os.path.join(dir,file) + " -> " + os.path.join(new_dir,file)
            shutil.copy(os.path.join(dir,file),os.path.join(new_dir,file))
    
###############################################################################
# Main
###############################################################################

def main():
    from config import ErosConfig
    config = ErosConfig()
    from optparse import OptionParser
    usage = "\n\
  %prog               : shows the currently set ros toolchain\n\
  %prog clear         : clear the currently set ros toolchain\n\
  %prog create        : create a user-defined toolchain configuration\n\
  %prog delete        : delete a preconfigured toolchain\n\
  %prog help          : print this help information\n\
  %prog list          : list available eros and user-defined toolchains\n\
  %prog select        : interactively select a toolchain\n\
  %prog select <str>  : directly select the specified toolchain\n\
  %prog validate      : attempt to validate a toolchain (not yet implemented)\n\
  \n\
Description: \n\
  Create/delete and manage the toolchain configuration for this ros environment.\n\
  Location of the user toolchain directory can be modified via --dir or more \n\
  permanently via " + core.eros_config() + "."
    parser = OptionParser(usage=usage)
    parser.add_option("-d","--dir", action="store", default=config.user_toolchains_dir(), help="location of the user toolchain library")
    options, args = parser.parse_args()
    
    # Configure the user toolchain directory.
    user_toolchain_dir(options.dir)
        
    ###################
    # Show current
    ###################
    if not args:
        show_current_toolchain()
        return 0

    command = args[0]

    ###################
    # Help
    ###################
    if command == 'help':
        parser.print_help()
        return 0
        
    ###################
    # List
    ###################
    if command == 'list':
        list_toolchains()
        return 0
    ###################
    # Clear
    ###################
    if command == 'clear':
        if os.path.exists(core.rostoolchain_cmake()):
            os.remove(core.rostoolchain_cmake())
            print
            print "-- Toolchain configuration cleared."
            # print "-- Remember to reconfigure ROS_BOOST_ROOT if necessary."
            print
        else:
            print
            print "-- Nothing to do (no toolchain configuration present)."
            print
        return 0
    ###################
    # Create
    ###################
    if command == 'create':
        return create_toolchain()

    ###################
    # Delete
    ###################
    if command == 'delete':
        return delete_toolchain()

    ###################
    # Select
    ###################
    if command == 'select': 
        if len(args) == 1: # interactive selection
            if not select_toolchain():
                return 1
        else:
            if args[1].isdigit():
                if select_toolchain_by_id(args[1]):
                    return 1
            else:
                if select_toolchain_by_name(args[1]):
                    return 1
        # Not currently needing it, but anyway, its good to have.
        print "-- Toolchain copied to rostoolchain.cmake."
        patch_ros()
        check_platform()
        #print "-- You need to manually export a root for the boost in your toolchain in setup.sh."
        #print "  -- (typically the same as the TOOLCHAIN_INSTALL_PREFIX), e.g."
        #print
        #print "          export ROS_BOOST_ROOT=\"/usr/my_toolchain_tuple/usr/local\""
        print 
        return 0
    
    ###################
    # Validate
    ###################
    if command == 'validate':
        print
        print "-- This command is not yet available."
        print
        return 0 
    
    # If we reach here, we have not received a valid command.
    print
    print "-- Not a valid command [" + command + "]."
    print
    parser.print_help()
    print

    return 1

if __name__ == "__main__":
    sys.exit(main())

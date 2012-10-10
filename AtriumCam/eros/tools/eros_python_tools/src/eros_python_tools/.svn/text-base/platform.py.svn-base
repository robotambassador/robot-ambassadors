'''
Created on Sep 7, 2010

Handles management of ros' rosconfig.cmake with a particular focus to customising
for platform/cpu characteristics.

@author: Daniel Stonier
'''
###############################################################################
# Modules
###############################################################################

import os
import shutil
import sys
import roslib
import core
import prefix

###############################################################################
# Objects
###############################################################################

class Platform:
    '''
    Represents data about a platform:
      pathname : full pathname to the platform
      name : usually the filename minus the extension e.g. core2
      group : valid groups are eros||user
      family : the architecture of cpu's to which it belongs
      current : true if it is the currently configured ros platform
    '''

    def __init__(self, platform_dir, platform_pathname, platform_id):
        # e.g. 
        #  platform_dir = /home/snorri/.ros/eros/platforms
        #  platform_pathname = /home/snorri/.ros/eros/platforms/arm/arm1176jzf-s.cmake
        self.pathname = platform_pathname
        # Could check here, but if calling from platform_list, will always be arg 1 we want.
        tail = self.pathname.split(platform_dir)[1] # e.g. /arm/arm1176jzf-s.cmake 
        cmake_name = os.path.basename(tail) # e.g. arm1176jzf-s.cmake
        self.name = os.path.splitext(cmake_name)[0] # e.g. arm1176jzf-s
        if ( self.pathname.find(eros_platform_dir()) != -1 ):
            self.group = "eros"
        else:
            self.group = "user"
        self.family = os.path.dirname(platform_pathname).split(platform_dir)[1] # e.g. /arm
        self.family = os.path.split(self.family)[1] # remove dir separators e.g. arm
        if ( self.family == '' ):
            self.family = "unknown"
        platform_exists = os.path.exists(core.rosconfig_cmake())
        self.current = False
        if ( platform_exists ) :
            name_string = "PLATFORM_NAME \"" + self.name + "\""
            if name_string in open(core.rosconfig_cmake()).read():
                family_string = "PLATFORM_FAMILY \"" + self.family + "\""
                if family_string in open(core.rosconfig_cmake()).read():
                    self.current = True
        self.id = platform_id
#        print "Pathname: " + self.pathname
#        print "Name " + self.name
#        print "Group: " + self.group
#        if ( self.current ):
#            print "Current: True"
#        else:
#            print "Current: False"
#        print "Id: " + repr(self.id)

###############################################################################
# Supporting Methods
###############################################################################

def eros_rosconfig_tail():
    return os.path.join(roslib.packages.get_pkg_dir('eros_python_tools'),"templates","rosconfig.tail.cmake")

def eros_platform_template():
    return os.path.join(roslib.packages.get_pkg_dir('eros_python_tools'),"templates","rosconfig.template.cmake")

def eros_platform_dir():
    return os.path.join(roslib.packages.get_pkg_dir('eros_platforms'),"library")

# global variable for the user platforms dir, access via user_platform_dir()
_user_platforms_dir = os.path.join(core.eros_home(),"platforms")

def user_platform_dir(dir=None):
    global _user_platforms_dir
    if ( dir != None ):
        if ( not os.path.exists(dir) ):
            raise Exception('Specified directory does not exist.')
        _user_platforms_dir = dir
    return _user_platforms_dir

# global variable for the platform list, access via platform_list()
platforms = []

def platform_list():
    """
    Both provides access to and populates the platform list for both eros and user-defined
    platform libraries.
    """
    global platforms
    if len(platforms) != 0 :
        return platforms
    # else populate from eros and user-defined libraries
    id_counter = 1
    for root, unused_dirs, files in os.walk(eros_platform_dir()):
        for name in files:
            if name.endswith('.cmake'):
                platforms += [Platform(eros_platform_dir(),os.path.join(root,name), id_counter)]
                id_counter += 1
    for root, unused_dirs, files in os.walk(user_platform_dir()):
        for name in files:
            if name.endswith('.cmake'):
                platforms += [Platform(user_platform_dir(),os.path.join(root,name), id_counter)]
                id_counter += 1
    return platforms

###############################################################################
# Primary Methods
###############################################################################

def show_current_platform():
    '''
    Print the identity of the currently configured platform:
      - checks eros/user platform libraries for a match
      - if not eros/user platform, checks if platform configured, but unknown
      - otherwise prints none 
    '''
    pretext = core.bold_string("Current platform: ")
    platforms = platform_list()
    found = False
    for platform in platforms:
        if ( platform.current ):
            found = True
            current_platform = platform
    if ( found ):
        print pretext + current_platform.family + os.sep + current_platform.name
    else:
        if ( os.path.exists(core.rosconfig_cmake()) ):
            print pretext + "unknown"
        else:
            print pretext + "none"
    
def list_eros_platforms():
    platforms = platform_list()
    print
    print core.bold_string("Eros platforms:")
    print
    for platform in platforms:
        if ( platform.group == 'eros' ):
            if ( platform.current ):
                print "  %d) %s%s%s%s" %(platform.id,platform.family,os.sep,platform.name,core.red_string("*"))
            else:
                print "  %d) %s%s%s" %(platform.id,platform.family,os.sep,platform.name)

def list_user_platforms():
    platforms = platform_list()
    print
    print core.bold_string("User platforms:")
    print
    for platform in platforms:
        if ( platform.group == 'user' ):
            if ( platform.current ):
                print "  %d) %s%s%s%s" %(platform.id,platform.family,os.sep,platform.name,core.red_string("*"))
            else:
                print "  %d) %s%s%s" %(platform.id,platform.family,os.sep,platform.name)

def list_platforms():
    list_eros_platforms()
    list_user_platforms()
    print
    
def select_platform():
    '''
    Interactively selects and sets an ros platform.
      - return 1 if failure, 0 if success
    '''
    list_platforms()
    platform_id_string = raw_input("Enter a platform id #: ")
    if ( not platform_id_string.isdigit() ):
        print core.red_string("Aborting, invalid id #.")
        return 1
    platform_id = int(platform_id_string)
    platforms = platform_list()
    if not platform_id <= len(platforms):
        print core.red_string("Aborting, invalid id #.")
        return 1
    found_platform = False
    for platform in platforms:
        if ( platform.id == platform_id ):
            found_platform = True
            selected_platform = platform
            break
    if ( not found_platform ):
        print core.red_string("Aborting, platform not found.")
        return 1
    else:
        set_platform(selected_platform)
        return 0

def select_platform_by_name(id_string):
    '''
    Selects platform by name or family/name string.
      - return 1 if failure, 0 if success
    '''
    bits = os.path.split(id_string)
    if  len(bits) == 2 :
        family = bits[0] # if just name is given, this will be empty
        name = bits[1]
        found_platform = False
        platforms = platform_list()
        if family == "": # try and just match name
            for platform in platforms:
                if ( platform.name == name ):
                    if found_platform:
                        print
                        print core.red_string("Multiple matches, please provide a full id string.")
                        list_platforms()
                        return 1
                    else:
                        found_platform = True
                        selected_platform = platform
        else: # try and match both family, name
            for platform in platforms:
                if platform.family == family and platform.name == name:
                    found_platform = True
                    selected_platform = platform
                    break
        if ( not found_platform ):
            print
            print core.red_string("Aborting, platform not found.")
            list_platforms()
            return 1
        else:
            set_platform(selected_platform)
            return 0
    else:
        print
        print core.red_string("Aborting, platform not found.")
        list_platforms()
        return 1
        

def set_platform(platform):
        # Copy across the platform specific file and append the default section as well
        shutil.copyfile(platform.pathname,core.rosconfig_cmake())
        rosconfig_tail = open(eros_rosconfig_tail()).read()
        f = open(core.rosconfig_cmake(), 'a')
        f.write(rosconfig_tail)
        f.close()
        check_install_prefix()
        print "-- Platform configured (rosconfig.cmake)."
        print

def select_default():
    '''
    Quietly selects and generates the default (vanilla) platform configuration.
    '''
    pathname = os.path.join(eros_platform_dir(),"generic","vanilla.cmake")
    shutil.copyfile(pathname,core.rosconfig_cmake())
    rosconfig_tail = open(eros_rosconfig_tail()).read()
    f = open(core.rosconfig_cmake(), 'a')
    f.write(rosconfig_tail)
    f.close()
    check_install_prefix()
    
def delete_platform():
    '''
    Interactively deletes a user-defined platform.
      - return 1 if failure, 0 if success
    '''
    list_user_platforms()
    print
    platform_id_string = raw_input("Enter a platform id #: ")
    print
    if ( not platform_id_string.isdigit() ):
        print core.red_string("-- Aborting, invalid id #.")
        return 1
    platform_id = int(platform_id_string)
    platforms = platform_list()
    if not platform_id <= len(platforms):
        print core.red_string("-- Aborting, invalid id #.")
        return 1
    found_platform = False
    for platform in platforms:
        if ( platform.id == platform_id ):
            if ( platform.group == 'user' ):
                found_platform = True
                selected_platform = platform
    if ( not found_platform ):
        print core.red_string("-- Aborting, invalid id #.") # probably passed an eros platform id
        return 1
    else:
        os.remove(selected_platform.pathname)
        return 0

def create_platform():
    '''
    Create a platform configuration.
    '''
    print
    print core.bold_string("  Creating a User-Defined Eros Platform Configuration")
    print
    print "This is an interactive assistant to help define a new eros style cmake platform"
    print "configuration. It will prompt you for a few custom strings and then save the"
    print "configured platform module in ROS_HOME/eros/platforms (~/.ros/eros/platforms on linux)."
    print "It can then be listed and selected in the same way as as a regular eros platform"
    print "configuration."
    print
    print core.bold_string("  Platform Family")
    print 
    print "  This is simply a convenience variable that helps sort platforms in the eros"
    print "  and user-defined libraries. Common examples include: intel, arm etc."
    print
    platform_family = raw_input('  Enter a string for the platform family [custom]: ')
    if ( platform_family == '' ):
        platform_family = 'custom'
    print
    print core.bold_string("  Platform Name")
    print 
    print "  This is unique identifier usually denoting the specific cpu that it represents"
    print "  and will be the name of the resulting cmake file. Common examples include: "
    print "  core2, arm1176jzf-s etc."
    print
    platform_name = raw_input('  Enter a string for the platform name [generic]: ')
    print
    print core.bold_string("  Platform Compile flags")
    print 
    print "  A string containing any extra compile flags to be used for this configuration."
    print "  e.g. '-march=core2 -sse4. This is in addition to the default ros flags."
    print 
    platform_compile_flags = raw_input('  Enter a string for the platform compile flags []: ')
    print
    print core.bold_string("  Platform Link flags")
    print 
    print "  A string containing any extra link flags to be used for this configuration."
    print "  e.g. '-Wl,--as-needed'. This is in addition to the default ros flags."
    print 
    platform_link_flags = raw_input('  Enter a string for the platform link flags []: ')
    
    platform_template = open(eros_platform_template()).read()
    platform_template = platform_template.replace('${platform_family}',platform_family)
    platform_template = platform_template.replace('${platform_name}',platform_name)
    platform_template = platform_template.replace('${platform_compile_flags}',platform_compile_flags)
    platform_template = platform_template.replace('${platform_link_flags}',platform_link_flags)
    #print platform_template
    user_defined_platform_pathname = os.path.join(user_platform_dir(),platform_family,platform_name+'.cmake')
    if ( os.path.exists(user_defined_platform_pathname) ):
        print core.red_string("  Aborting, this platform configuration already exists (use --delete to remove).")
        return 1
    if not os.path.exists( os.path.dirname(user_defined_platform_pathname) ): # Make sure the dir exists before we open for writing
        os.makedirs(os.path.dirname(user_defined_platform_pathname))
    f = open(user_defined_platform_pathname, 'w')
    f.write(platform_template)
    rosconfig_tail = open(eros_rosconfig_tail()).read()
    f.write(rosconfig_tail)
    f.close()
    print
    print core.bold_string("Platform Finalised")
    print "-- Family: %s" %platform_family
    print "-- Name: %s" %platform_name
    print "-- CFlags: %s" %platform_compile_flags
    print "-- LFlags: %s" %platform_link_flags
    print "-- File: %s" %user_defined_platform_pathname
    print
    check_install_prefix()
    
def check_install_prefix():
    ''' 
    Runs a quick check to see if a toolchain is installed, and if so, 
    configures the install prefix to match the toolchain's install root.
    '''
    if core.is_rostoolchain_cmake():
        print "-- Found rostoolchain configuration (rostoolchain.cmake)."
        print "  -- Setting the install prefix to ${TOOLCHAIN_INSTALL_PREFIX}"
        prefix.set_install_prefix("${TOOLCHAIN_INSTALL_PREFIX}")
        print "  -- Confirm that it is compatible with the current toolchain."
        print "  -- Or modify it as you wish with 'rosprefix'."
    else:
        print "-- No rostoolchain found (rostoolchain.cmake)."
        print "  -- Setting the install prefix to /usr/local."
    
    
###############################################################################
# Main
###############################################################################

def main():
    from config import ErosConfig
    config = ErosConfig()
    from optparse import OptionParser
    usage = "\n\
  %prog               : shows the currently set ros platform\n\
  %prog clear         : clear the currently set ros platform\n\
  %prog create        : create a user-defined platform configuration\n\
  %prog delete        : delete a platform configuration\n\
  %prog help          : print this help information\n\
  %prog list          : list available eros and user-defined platforms\n\
  %prog select        : interactively select a platform configuration\n\
  %prog select <str>  : directly select the specified platform configuration\n\
  %prog validate      : attempt to validate a platform (not yet implemented)\n\
\n\
Description: \n\
  Create/delete and manage the platform configuration for this ros environment\n\
  Location of the user platform directory can be modified via --dir or more \n\
  permanently via " + core.eros_config() + "."
    parser = OptionParser(usage=usage)
    parser.add_option("-d","--dir", action="store", default=config.user_platforms_dir(), help="location of the user platforms directory.")
    #parser.add_option("-v","--validate", action="store_true", dest="validate", help="when creating, attempt to validate the configuration")
    options, args = parser.parse_args()
    
    # Configure the user platform directory.
    user_platform_dir(options.dir)

    ###################
    # Show current
    ###################
    if not args:
        show_current_platform()
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
        list_platforms()
        return 0
    
    ###################
    # Clear
    ###################
    if command == 'clear':
        if os.path.exists(core.rosconfig_cmake()):
            os.remove(core.rosconfig_cmake())
            print
            print "-- Platform configuration cleared."
            print
        else:
            print
            print "-- Nothing to do (no rosconfig.cmake)."
            print
        return 0
    ###################
    # Create
    ###################
    if command == 'create':
        return create_platform()

    ###################
    # Delete
    ###################
    if command == 'delete':
        return delete_platform()

    ###################
    # Select
    ###################
    if command == 'select':
        if len(args) == 1: # interactive selection
            if not select_platform():
                return 1
        else:
            if not select_platform_by_name(args[1]):
                return 1
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

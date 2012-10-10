'''
Created on Sep 9, 2010

Used to set the build mode for an ros environment (via rosconfig.cmake)

@author: Daniel Stonier
'''

import os
import sys
import fileinput
import platform
import core

def show_current_build_mode():
    pretext = core.bold_string("Current build mode: ")
    found = False
    if os.path.exists(core.rosconfig_cmake()):
        for line in fileinput.input(core.rosconfig_cmake(),mode='r'):
            if (line.find("set(ROS_BUILD_TYPE Debug") != -1):
                sys.stdout.write(pretext + "Debug\n")
                found = True
                break
            elif (line.find("set(ROS_BUILD_TYPE Release") != -1):
                sys.stdout.write(pretext + "Release\n")
                found = True
                break
            elif (line.find("set(ROS_BUILD_TYPE RelWithDebInfo") != -1):
                sys.stdout.write(pretext + "RelWithDebInfo\n")
                found = True
                break
            elif (line.find("set(ROS_BUILD_TYPE MinSizeRel") != -1):
                sys.stdout.write(pretext + "MinSizeRel\n")
                found = True
                break
        if not found:
            print core.red_string("Unknown") + "(please check ROS_ROOT/rosconfig.cmake for problems)."
    else:
        print pretext + "RelWithDebInfo"

def create_vanilla_config():
#    print "No existing platform configuration exists - creating a default."
    platform.select_default()
    
def set_debug_mode(mode):
    if not validate_mode(mode):
        return 1
    if not os.path.exists(core.rosconfig_cmake()):
        create_vanilla_config()
    for line in fileinput.input(core.rosconfig_cmake(),inplace=1,mode='r'):
        if ( line.find("set(ROS_BUILD_TYPE") != -1):
            line = "  set(ROS_BUILD_TYPE "+mode+")"
        if line.endswith("\n"):
            line = line[:-1]
        print line
    print core.bold_string("New build mode: ") + mode
    return 0

def print_valid_modes():
    print core.bold_string("Available Build Modes")
    print "  Debug"
    print "  RelWithDebInfo"
    print "  Release"
    print "  MinSizeRel"
    
def validate_mode(mode):
    if (mode != "Debug" ) and ( mode != "Release" ) and ( mode != "RelWithDebInfo" ) and ( mode != "MinSizeRel" ):
        print_valid_modes()
        print(core.red_string(mode) + " is not a valid build mode type.")
        return False
    return True
    
def main():
    from optparse import OptionParser
    usage = "\n\
  %prog : display the current build mode.\n\
  %prog list : list valid build modes.\n\
  %prog help : print this help information.\n\
  %prog <Debug|Release|RelWithDebInfo|MinSizeRel> : set the build mode.\n\
\n\
Description: \n\
  Use to configure the global build mode for this ros environment."
    parser = OptionParser(usage=usage)
    unused_options, args = parser.parse_args()
    
    ###################
    # Default/Current
    ###################
    if not args:
        show_current_build_mode()
        sys.exit(0)

    ###################
    # Commands
    ###################
    if args[0] == "list":
        print_valid_modes()
        sys.exit(0)

    if args[0] == "help":
        parser.print_help()
        sys.exit(0)
    
    ###################
    # Set
    ###################
    mode = args[0]
    return set_debug_mode(mode)

if __name__ == "__main__":
    sys.exit(main())

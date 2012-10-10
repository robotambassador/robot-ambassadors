'''
Created on 17 Aug, 2010

Various routines which protect the ros stacks from recompilation
via the use of the ROS_NOBUILD file flag.

@author: Daniel Stonier
'''
###############################################################################
# Imports
###############################################################################

import os
import sys
import subprocess
import roslib
import core
from eros_python_tools.core import ros_version

###############################################################################
# Objects
###############################################################################

class Flags:
    '''
    Structure for storing the status of various build flags.
    '''
    unprotect = False
    clean = False
    rosdeps = False

###############################################################################
# Methods
###############################################################################

def minimal_package_set():
    ''' Define the minimal set of packages needed for a cross compiled ros runtime that can run ros clients and server. '''
    version = ros_version()
    if ( version == "boxturtle" ):
        packages = ["rospack", "genmsg_cpp", "roslang", "xmlrpcpp", "pycrypto", "paramiko", "roslib", "rospy", "rosconsole", "roscpp", "rosout", "roslaunch", "rostest"]
    elif ( version == "cturtle" ):
        packages = ["rospack", "genmsg_cpp", "rosbag", "rosclean", "rosdoc", "rosgraph", "roslang", "roslib", "rosmaster", "rosmsg", "rospy", "rosconsole", "roscpp", "rosout", "roslaunch", "rostest", "rxdeps", "topic_tools", "xmlrpcpp"]
    elif ( version == "diamondback" ):
        packages = ["rospack", "rosbuild", "cpp_common", "roslib", "roslang", "xmlrpcpp", "rosgraph_msgs", "roscpp_traits", "rostime", "std_msgs", "roscpp_serialization", "rosconsole", "roscpp", "rosclean", "rosgraph", "rosmaster", "rosunit", "rospy", "rosout", "roslaunch", "rostest", "topic_tools", "rosbag", "rosmsg" ]
    else:
        print "Unsupported version - defaulting to build/protect the latest official release's minimal set (diamondback)."
        packages = ["rospack", "rosbuild", "cpp_common", "roslib", "roslang", "xmlrpcpp", "rosgraph_msgs", "roscpp_traits", "rostime", "std_msgs", "roscpp_serialization", "rosconsole", "roscpp", "rosclean", "rosgraph", "rosmaster", "rosunit", "rospy", "rosout", "roslaunch", "rostest", "topic_tools", "rosbag", "rosmsg" ]
    return packages


def comms_package_set():
    '''
    Defines the minimum set of packages required for cpp communications. Only 
    supported for diamondback and later.
    '''
    version = ros_version()
    if ( version == "boxturtle" ):
        print "Boxturtle is unsupported (req. diamondback+)."
        packages = []
    elif ( version == "cturtle" ):
        print "Cturtle is unsupported (req. diamondback+)."
        packages = []
    elif ( version == "diamondback" ):
        packages = ["rosbuild", "roslang", "cpp_common", "roscpp_traits", "rostime", "roscpp_serialization", "rospack", "roslib", "xmlrpcpp", "rosconsole", "std_msgs", "rosgraph_msgs", "roscpp" ]
    else:
        print "Unsupported version - defaulting to build/protect the latest official release's comms set (diamondback)."
        packages = ["rosbuild", "roslang", "cpp_common", "roscpp_traits", "rostime", "roscpp_serialization", "rospack", "roslib", "xmlrpcpp", "rosconsole", "std_msgs", "rosgraph_msgs", "roscpp" ]
    return packages

def stacks_package_set(stacks):
    '''
    Gather all the packages in the given list of stack.
    '''
    packages = []
    for stack in stacks:
        print stack
        packages += core.rosstack_packages(stack)
    return packages

# Often we need to kickstart the process manually (without rospack, we can't do rosmake!)
def make_rospack():
    ''' Cleans and recompiles rospack. '''
    rospack_dir = os.getenv("ROS_ROOT")+"/tools/rospack"
    unused_retcode = subprocess.call(["make", "clean"],cwd=rospack_dir)
    unused_retcode = subprocess.call(["make"],cwd=rospack_dir)

def add_package_dependencies(packages):
    '''
    Make sure each package has both itself and all its dependencies included
    in the package list.
    '''
    all_packages = []
    for package in packages:
        all_packages += [package]
        all_packages += roslib.rospack.rospack_depends(package)
    return all_packages
         
def protect(packages):
    '''
    Check the flags and appropriately apply the correct rosmake call
    to protect the selected packages. 
    '''
    if Flags.unprotect:
        p = subprocess.Popen(["rosmake", "-u"] + packages)
        p.wait()
        return 0
    rosmake_args = [] 
    if Flags.clean:
        make_rospack()
        rosmake_args += ["--pre-clean"]
    if Flags.rosdeps:
        rosmake_args += ["--rosdep-install", "--rosdep-yes"]
    p = subprocess.Popen(["rosmake", "-i"] + rosmake_args + packages)
    p.wait()
    return 0

def main():
    ''' Main entry point for creating an rosprotect script.'''
    from optparse import OptionParser
    usage = "\n\
  %prog [options] targets : protect/unprotect the specified targets\n\
  %prog help              : print this help information.\n\
\n\
Description:\n\
  Protect/unprotect ros packages via the ROS_NOBUILD mechanism. Targets can include\n\
  a package list, stack list or one of the preconfigured groups (e.g. --minimal)." 
    parser = OptionParser(usage=usage)
    parser.add_option("-x","--clean", action="store_true", dest="clean", default=False, help="depracated, use --pre-clean instead.")
    parser.add_option("-p","--pre-clean", action="store_true", dest="pre_clean", default=False, help="pre-clean packages before building and protecting them.")
    parser.add_option("-c",  "--comms", action="store_true", dest="comms", default=False, help="target the set of package required only for cpp communications.")
    parser.add_option("-m","--minimal", action="store_true", dest="minimal", default=False, help="target a selected minimal set for a minimal ros environment.")
    parser.add_option("-r","--rosdeps", action="store_true", dest="rosdeps", default=False, help="pass --rosdep-install and rosdep-yes to rosmake when protecting.")
    parser.add_option("-s","--stacks", action="store_true", dest="stacks", default=False, help="target stacks instead of packages.")
    parser.add_option("-u","--unprotect", action="store_true", dest="unprotect", default=False, help="unprotect packages instead of protecting them.")
    options, args = parser.parse_args()

    ###################
    # Abort Check
    ###################
    if ( not options.comms and not options.minimal and not args ):
        print
        parser.print_help()
        print    
        return 1

    ###################
    # Flags
    ###################
    if options.unprotect:
        Flags.unprotect = True
    
    if options.clean:
        print
        print "Depracated argument, use --pre-clean instead."
        print
        return 1

    if options.pre_clean:
        Flags.clean = True

    ###################
    # Commands
    ###################
    if len(args) != 0 and args[0] == "help":
        parser.print_help()
        sys.exit(0)

    ###################
    # Packages
    ###################
    if options.minimal:
        packages = minimal_package_set()
    elif options.comms:
        packages = comms_package_set()
    elif options.stacks:
        packages = stacks_package_set(args)
    else:
        packages = args
        
    if len(packages) == 0:
        print "-- No packages to protect (check your input arguments)."
        parser.print_help()
        return 1

    packages = add_package_dependencies(packages)
    return protect(packages)
        
if __name__ == "__main__":
    sys.exit(main())

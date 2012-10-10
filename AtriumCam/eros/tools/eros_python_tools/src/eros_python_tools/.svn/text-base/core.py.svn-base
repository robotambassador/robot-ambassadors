'''
Created on 17/08/2010

Various common core functions used by eros.

@author: Daniel Stonier
'''

import os
import re
import sys
import subprocess
import roslib

class Console:
    bold = "\033[1m"
    reset = "\033[0;0m"
    red = "\033[31m"

def red_string(msg):
    """bound string with console symbols for red output"""
    return Console.red + msg + Console.reset

def bold_string(msg):
    """bound string with console symbols for bold output"""
    return Console.bold + msg + Console.reset

def ros_version():
    rosversion_exe = roslib.rosenv.get_ros_root()+"/bin/rosversion"
    rosversion_exists = os.path.exists(rosversion_exe)
    if ( not rosversion_exists ):
        print "Could not find rosversion - make sure ROS_ROOT is set appropriately."
        sys.exit(1)
    version = subprocess.Popen([rosversion_exe, "ros"], stdout=subprocess.PIPE).communicate()[0]
    if ( re.match(r"1\.0.*", version) ): # Boxturtle
        version = "boxturtle"
    elif ( re.match(r"1\.2.*", version) ): # Cturtle
        version = "cturtle"
    elif ( re.match(r"1\.4.*", version) ): # Cturtle
        version = "diamondback"
    else: 
        version = "unknown"
    return version

def rostoolchain_cmake():
    return os.path.join(roslib.rosenv.get_ros_root(),"rostoolchain.cmake")

def is_rostoolchain_cmake():
    return os.path.exists(rostoolchain_cmake())

def rosconfig_cmake():
    return os.path.join(roslib.rosenv.get_ros_root(),"rosconfig.cmake")

def rosstack_packages(stack):
    return roslib.rospack.rosstackexec(['contents',stack]).split()

def eros_home():
    return os.path.join(roslib.rosenv.get_ros_home(),"eros")
    
def eros_config():
    return os.path.join(eros_home(),"eros.cfg")

def eros_tools_templates_dir():
    return os.path.join(roslib.packages.get_pkg_dir('eros_python_tools'),"templates")

# Finds and reads one of the templates.
def read_template(tmplf):
    #p = os.path.join(roslib.packages.get_pkg_dir('eros_python_tools'), tmplf)
    f = open(tmplf, 'r')
    try:
        t = f.read()
    finally:
        f.close()
    return t

# This inserts the labelled variables into the template wherever the corresponding
# %package, %brief, %description and %depends is found.
def instantiate_template(template, package, brief, description, author, depends):
    return template%locals()

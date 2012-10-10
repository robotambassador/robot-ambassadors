'''
Created on Sep 11, 2010

Used to set the install prefix for an ros environment (via rosconfig.cmake)

@author: Daniel Stonier
'''
import os
import sys
import fileinput
import platform
import core

def get_install_prefix():
    prefix = None
    if os.path.exists(core.rosconfig_cmake()):
        file = core.rosconfig_cmake()
        for line in fileinput.input(file,mode='r'):
            if ( line[0] != '#'):
                index = line.find("set(CMAKE_INSTALL_PREFIX")
                if (index != -1):
                    cache_index = line.find("CACHE")
                    prefix = line[index+len("set(CMAKE_INSTALL_PREFIX")+1:cache_index-1]
    return prefix

def show_current_install_prefix():
    prefix = get_install_prefix()
    if ( prefix is not None ):
        print core.bold_string("Current install prefix:") + " " + prefix
    else:
        print core.bold_string("Current install prefix:") + " not configured (using cmake default)."


def create_vanilla_config():
#    print "No existing platform configuration exists - creating a default."
    platform.select_default()
    
def set_install_prefix(prefix):
    '''
    Sets a new install prefix:
    - checks for an rosconfig.cmake, creates if not already present.
    - parses for cmake's install prefix command.
      - if present, modify the existing prefix.
      - if not present, its a custom config, simply insert a new install prefix.
    @return result : 0,1,2 depending on what was done (moved logging to outside because toolchain calls this function)
    '''
    result = 0
    if not os.path.exists(core.rosconfig_cmake()):
        create_vanilla_config()
        result = 1
    updated = False
    for line in fileinput.input(core.rosconfig_cmake(),inplace=1,mode='r'):
        if ( line.find("set(CMAKE_INSTALL_PREFIX") != -1):
            line = 'set(CMAKE_INSTALL_PREFIX '+prefix+' CACHE PATH "Install location" FORCE)'
            updated = True
        if line.endswith("\n"):
            line = line[:-1]
        print line
    if not updated:
        result = 2
        f = open(core.rosconfig_cmake(), 'a')
        line = 'set(CMAKE_INSTALL_PREFIX '+prefix+' CACHE PATH "Install location" FORCE)'
        f.write(line)
    return result

def main():
    from optparse import OptionParser
    usage = "\n\
  %prog : display the current install prefix.\n\
  %prog help : print this help information.\n\
  %prog <path> : set the new install prefix to <path>.\n\
\n\
Description: \n\
  Use to configure the global install prefix for this ros environment."
    parser = OptionParser(usage=usage)
    unused_options, args = parser.parse_args()
    
    ###################
    # Default/Current
    ###################
    if not args:
        show_current_install_prefix()
        return 0

    ###################
    # Commands
    ###################
    if args[0] == "help":
        parser.print_help()
        return 0
    
    ###################
    # Set
    ###################
    install_prefix = args[0]
    old_prefix = get_install_prefix()
    if old_prefix == None:
        old_prefix = "None"
    print ""
    result = set_install_prefix(install_prefix)
    print "-- Configuring a new install prefix."
    if ( result == 1 ):
        print "  -- No rosconfig.cmake, inserting a default (vanilla) eros configuration."
    elif( result == 2 ):
        print "  -- Custom rosconfig.cmake exists, but has no install prefix configuration."
        print "  -- Adding new install prefix to the custom rosconfig.cmake."
    print "-- Install prefix updated, " + old_prefix + " -> " + install_prefix
    print "" 

    return 0

if __name__ == "__main__":
    sys.exit(main())

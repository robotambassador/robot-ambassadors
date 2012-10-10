'''
Created on 28/09/2010

Handles the eros configuration file.

@author: Daniel Stonier
'''

import os
import sys
import core
import ConfigParser

###############################################################################
# Interface [ErosConfig]
###############################################################################

class ErosConfig:
    '''
    Represents configuration for the eros environment. The methods are the 
    means to access the configuration context.
    '''

    def __init__(self):
        self.check_for_eros_configuration()
        self.config = ConfigParser.RawConfigParser()
        self.config.read(os.path.join(core.eros_home(),"eros.cfg"))

    def check_for_eros_configuration(self):
        '''
        Creates a default configuration if it doesn't already exist.
        '''
        if(not os.path.exists(core.eros_home())):
            os.mkdir(core.eros_home())
        if(not os.path.exists(os.path.join(core.eros_home(),"toolchains"))):
            os.mkdir(os.path.join(core.eros_home(),"toolchains"))
        if(not os.path.exists(os.path.join(core.eros_home(),"platforms"))):
            os.mkdir(os.path.join(core.eros_home(),"platforms"))
        if (not os.path.exists(core.eros_config())):
            config = ConfigParser.RawConfigParser()
            config.add_section('Platforms')
            config.add_section('Toolchains')
            config.set('Platforms', 'user_platforms_dir', os.path.join(core.eros_home(),"platforms"))
            config.set('Toolchains', 'user_toolchains_dir', os.path.join(core.eros_home(),"toolchains"))
            # Writing the configuration file
            with open(core.eros_config(), 'wb') as configfile:
                config.write(configfile)
                
    def user_toolchains_dir(self):
        return self.config.get("Toolchains",'user_toolchains_dir')

    def user_platforms_dir(self):
        return self.config.get("Platforms",'user_platforms_dir')
        
    def new_user_toolchains_dir(self,new_toolchains_dir):
        self.config.set('Toolchains', 'user_toolchains_dir', new_toolchains_dir)
        self.write_to_file()

    def new_user_platforms_dir(self,new_platforms_dir):
        self.config.set('Platforms', 'user_platforms_dir', new_platforms_dir)
        self.write_to_file()
        
    def write_to_file(self):
        with open(core.eros_config(), 'wb') as configfile:
            self.config.write(configfile)

def unit_test():
    print "Testing the eros configuration file parser."
    config = ErosConfig()
    print "  Platforms Dir : %s" %config.user_platforms_dir() 
    print "  Toolchains Dir: %s" %config.user_toolchains_dir()
    config.new_user_platforms_dir("/opt/ros/ycs/platforms") 
    config.new_user_toolchains_dir("/opt/ros/ycs/toolchains") 
    config.new_user_platforms_dir(os.path.join(core.eros_home(),"toolchains")) 
    config.new_user_toolchains_dir(os.path.join(core.eros_home(),"platforms"))
    
if __name__ == "__main__":
    sys.exit(unit_test())

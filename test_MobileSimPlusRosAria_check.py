#!/usr/bin/env python
# Copyright 2017-2018 by University of Cincinnati
# Copyright 2015-2016 by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite
"""
=== Summary of contents ===
"""

# Note that the following may only work if you run this python script from the directory in which it resides...
#
# This is done so you can use the import command on other/separate modules (this is adding it to the beginning like it should've automagically done for you); remember to create an empty __init__.py in the local directory for this to load properly
import os
import sys # for sys.exit() and sys.path.append()
sys.path.append(os.getcwd()) # modify sys.path to include current directory
sys.path.append(os.getcwd() + '/common') # modify sys.path to include ./common directory
import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
import rosConnectWrapper as rC
import ws4pyFunctions as wsF
import fileFunctions as fF

from time import sleep

if __name__ == '__main__':
    print("sys.argv = " + str(sys.argv))

    # python test_ws4pyCheck.py connection ...
    argvdefaults = ["ws://localhost:9090/"] # default connection valid because starting rosbridge from this file :)
    [connection] = fF.readArgvpieces(sys.argv,argvdefaults)
        
    try:
        # example code for attempting to check if can reach rosbridge server without erroring out on publish and/or param server read
        #
        #print("Force wait until rosbridge interface and parameter server comes online...")
        #testReturn = wsF.generalRecvParamServer(connection,'/',['rosdistro'],'block')
        #print("Parameter server is up and responding!")
        ##print("testReturn = %s" % str(testReturn))
        ##sleep(3)
        print("Force wait until rosbridge interface and parameter server AND MOBILESIM + ROSARIA comes online...")
        testReturn1 = wsF.generalRecvTopicsServicesList(connection,"/rosapi/topics",'/RosAria/cmd_vel','block')
        print("Parameter server is up and responding, and so are MobileSim and RosAria!")
        #print("testReturn2 = %s" % str(testReturn3))
        #print("Done-good. yay?")
    except: # KeyboardInterrupt:
        #print("we hit Ctrl-C")
        print("Done-bad. boo!")

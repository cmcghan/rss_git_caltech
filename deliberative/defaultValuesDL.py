# Copyright 2017-2018 by University of Cincinnati
# Copyright 2014-2016 by California Institute of Technology
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
sys.path.append(os.getcwd() + '/../common') # modify sys.path to include ../common directory
from waypointFunctions import getwaypointFile
#from getConnectionIPaddress import getConnectionIPaddress # in ../common

def menuDL(echo_input):
    print("(pseudo-)Deliberative layer (python)")
    print("------------------------------------")
    
    if (echo_input == -1):
        echo_input = int(raw_input("Please enter runtime mode (1=OTHER points 2 (HL-passthru), 2=operator waypoints(-only), 3=user input, 4=undef., 5=undef., 6=operator msg, 7=OTHER points 1 (HL-planning)) (default = 1)?: "))
    if (echo_input > 7) or (echo_input < 1):
        echo_input = 1

    waypointsOnly = False # for now, support the old waypoint-only format as default...
    if (echo_input == 1):
        waypointFile = getwaypointFile(1)
    elif (echo_input == 2):
        waypointFile = getwaypointFile(0)
        waypointsOnly = True
    elif (echo_input == 3):
        pass
    elif (echo_input == 4):
        pass
    elif (echo_input == 5):
        pass
    elif (echo_input == 6):
        waypointFile = getwaypointFile(0)
    elif (echo_input == 7):
        waypointFile = getwaypointFile(2)
    else:
        echo_input = 3 # ask user for input

    if (echo_input == 3): # get [waypointFile] now
        waypointFile = getwaypointFile(None) # ask user for value
        if (waypointFile is None):
            waypointsOnlyHold = int(raw_input("Please enter 1 for operator receive to be waypoints-only, 0 for full data (default=0): "))
            if (waypointsOnlyHold == 1):
                waypointsOnly = True
            else:
                waypointsOnly = False
        
    values = [waypointFile,
              waypointsOnly]
    if (echo_input != 3): # print out values being used if defaults
        print("values being used:\n"
            +"[waypointFile,"
            +" waypointsOnly]\n = %r" % values)

    return values

def constRuntimeParamsDL(robotType,simType):
    if ([robotType,simType] == ['p3dx','MobileSim']):
        stepTime = 0.01
    elif ([robotType,simType] == ['p3dx','hardware']):
        stepTime = 0.01
    elif ([robotType,simType] == ['p3dx','gazebo']):
        stepTime = 0.01
    else:
        print("Error, unknown [robotType,simType] pair: ['%s','%s']. Exiting." % (robotType,simType))
        sys.exit(0)
    
    return [stepTime]

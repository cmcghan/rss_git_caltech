#! /usr/bin/env python
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
sys.path.append(os.getcwd() + '/common') # modify sys.path to include ./common directory
import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
import rosConnectWrapper as rC
import getConnectionIPaddress as gC

from time import sleep

if __name__ == '__main__':
    connection = gC.getConnectionIPaddress()
    waypointsOnlyHold = int(raw_input("Please enter 1 for operator receive to be waypoints-only, 0 for full data (default=0): "))
    if (waypointsOnlyHold == 1):
        waypointsOnly = True
    else:
        waypointsOnly = False
    try:
        # example code for sending waypoints to pseudoDL
        #
        if (waypointsOnly == True):
            ws_OptoDL_out = rC.RosMsg('ws4py', connection, 'pub', '/operator/waypoint_list', 'nav_msgs/Path', ws4pyROS.pack_waypoints_ignorerest) # for consistency
        else: # if(waypointsOnly == False):
            ws_OptoDL_out = rC.RosMsg('ws4py', connection, 'pub', '/operator/Op_to_DL', 'rss_msgs/OptoDL', ws4pyROS.pack_OptoDL)
        
        sleep(3)
        waypoints_to_send = [ [1.0, 2.0], [7.0, 2.0], [6.0, 3.0], [-1.0, 2.0], [1.0, 2.0] ] # list of lists
        ws_OptoDL_out.send_pieces([[waypoints_to_send, 1],-1, 0]) # send waypoint(s), waytype=1 ([x,y] points, no set orientation), risk = -1 (n/a, ignored), handling = 0 (drop all else and do this)
                
        sleep(3)
    except KeyboardInterrupt:
        ws_OptoDL_out.closeNow()

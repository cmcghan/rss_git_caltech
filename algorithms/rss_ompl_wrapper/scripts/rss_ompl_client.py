#! /usr/bin/env python
# Copyright 2017-2018 by University of Cincinnati
# Copyright 2014-2016 by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite
"""
Simple test script for raster_planning
"""
#from time import sleep

# Note that the following may only work if you run this python script from the directory in which it resides...
#
# This is done so you can use the import command on other/separate modules (this is adding it to the beginning like it should've automagically done for you); remember to create an empty __init__.py in the local directory for this to load properly
import os
import sys # for sys.exit() and sys.path.append()
#sys.path.append(os.getcwd()) # modify sys.path to include current directory
sys.path.append(os.getcwd() + '/../../../common') # modify sys.path to include ../common directory
sys.path.append(os.path.dirname(__file__) + '/../../../common') # modify sys.path to include ../common directory # for run using rosrun

import rastermapFunctions as rmF # in ../common
import ws4pyFunctions as wsF # in ../common

#from __future__ import print_function
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import rss_ompl_wrapper.msg

def rss_ompl_client(start,end,OMPLsolverData,growlen,rParams,plannerstr,pts_calcd):
    # needs this for client.wait_for_result() apparently?
    print("Starting rospy node...")
    rospy.init_node('rss_ompl_client_py')
    print("Set up complete!")
        
    # create client
    client = actionlib.SimpleActionClient('rss_ompl_actionNode', rss_ompl_wrapper.msg.rssomplactionAction)
    
    # wait until action server is up and running, returns once it is
    print("Waiting for action server to come online...")
    client.wait_for_server()
    print("Action server is online!")
    
    # putting together goal to send to action server
    [intmap,rover,timetry,retries] = OMPLsolverData
    #need to convert np.array() intmap to list of lists before string conversion, because opposite side won't eval/convert it (the numpy array) properly with the function we're using
    holdmap = [];
    for row in intmap:
        holdrow = []
        for col in row:
            holdrow.append(col)
        holdmap.append(holdrow)
    #print("Current location / Starting point is %r globally and %r on raster map" % (start,rstart))
    rstart = rmF.globalToRaster(start,rParams)
    rend = rmF.globalToRaster(end,rParams)
    #print("Goal / Ending point is %r globally and %r on raster map" % (end,rend))
    goal = rss_ompl_wrapper.msg.rssomplactionGoal(rstart=rstart,rend=rend,intmap=str(holdmap),growlen=growlen,timetry=timetry,retries=retries,rParams=rParams,plannerstr=plannerstr,pts_calcd=pts_calcd)
    
    print("Sending goal to action server...")
    client.send_goal(goal)
    print("Waiting for result from server...")
    client.wait_for_result()
    print("Result received!")
    theresult = client.get_result()
    planstatus = theresult.planstatus
    rpath = wsF.convertStringToInternalType(theresult.rpath)
    return [planstatus,rpath]

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        #print("Starting node...")
        #rospy.init_node('rss_ompl_client_py')
        #print("Node is now set up!")
        print("Starting client (to ask for result)...")
        #[intmap,rParams,rover] = rmF.rasterMapSetup(mapname,connection,thresh,rwidth,rheight,growlen)
        env_mat = rmF.readMapFromCsvFile(os.path.dirname(__file__) + '/../../../rss_maps/obstacles.csv')
        [rwidth,rheight,thresh,growlen] = [100,100,None,0]
        mapcorners = [-15,15,15,-15] # ['upperLeftX','upperLeftY','lowerRightX','lowerRightY']
        rParams = rmF.mapParams(mapcorners,rwidth=rwidth,rheight=rheight,gwidth=30.0,gheight=30.0)
        intmap = rmF.thresholdCsvMap(env_mat,thresh,[rwidth,rheight]) # csvmap = env_mat
        OMPLsolverData = [intmap,None,5,3] # [intmap,rover,timetry,retries] # rover not used...
        [planstatus,rpath] = rss_ompl_client([-13,13],[13,-13],OMPLsolverData,growlen,rParams,'RRTstar',0) # result
        print("Result received!")
        if (planstatus):
            print("Result (raster):", ', '.join([str(n) for n in rpath]))
            path = [rmF.rasterToGlobal(rpoint,rParams) for rpoint in rpath]
            print("Result (global):", ', '.join([str(n) for n in path]))
        else:
            print("Error, path not found.")
        print("All done! :)")
    except rospy.ROSInterruptException:
        #print("program interrupted before completion", file=sys.stderr)
        print("Error: program interrupted before completion")

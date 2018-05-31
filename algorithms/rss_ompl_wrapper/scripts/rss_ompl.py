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

import rospy

import actionlib

import rss_ompl_wrapper.msg

class RSSOMPLAction(object):
    # create messages that are used to publish feedback/result
    _feedback = rss_ompl_wrapper.msg.rssomplactionFeedback()
    _result = rss_ompl_wrapper.msg.rssomplactionResult()

    def __init__(self, name):
        print("Starting __init__...")
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rss_ompl_wrapper.msg.rssomplactionAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        print("__init__ done!")
      
    def execute_cb(self, goal):
        print("Starting execute_cb...")
        #success = True
        planstatus = False
        
        rstart = goal.rstart
        rend = goal.rend
        import numpy as np
        intmap = np.array(wsF.convertStringToInternalType(goal.intmap))
        #rover = goal.rover
        growlen  = goal.growlen
        timetry = goal.timetry
        retries = goal.retries
        rParams = goal.rParams
        plannerstr = goal.plannerstr
        pts_calcd = goal.pts_calcd
        #import rasterplanningOMPL as raster_planning # in ../common
        import rasterplanningOMPL111 as raster_planning # in ../common
        import polytope as pc
        rXscale = rParams[0]; rYscale = rParams[1] # [rXscale,rYscale,rXoffset,rYoffset,rXsign,rYsign] = rParams
        rover = pc.box2poly([[-0.25*rXscale-growlen, 0.25*rYscale+growlen], [-0.15*rXscale-growlen, 0.15*rYscale+growlen]]) # fixed rover size, 0.25 in x (scaled plus buffer), 0.15 in y (scaled plus buffer)
        rEnv = raster_planning.Raster_Env(np.array(intmap), rover) # need to create new variable?
        
        rospy.loginfo('%s: Executing, starting at (raster) %r with goal %r, (timetry,retries)=(%r,%r) and plannerstr = %s' % (self._action_name,rstart,rend,timetry,retries,plannerstr))
        
        # check if "preempt" was requested by client
        #if self._as.is_preempt_requested():
        #    rospy.loginfo('%s: Preempted' % self._action_name)
        #    self._as.set_preempted()
        #    #success = False
        #    planstatus = False
        planstatus = rEnv.plan(rstart, rend, 0, timetry, retries, plannerstr) # third number is 0 for first call(/here)
        # publish the feedback
        #self._as.publish_feedback(self._feedback)
        
        #if success:
        if planstatus:
            rpath = rEnv.record_solution('raster_path'+str(pts_calcd)) # retrieve path in raster coords; also prints rpath to raster_path
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._result.planstatus = planstatus
            self._result.rpath = str(rpath)
            self._as.set_succeeded(self._result)
        else:
            self._result.planstatus = planstatus
            self._result.rpath = str([])
            self._as.set_aborted(self._result) # failed out, in this case...
        print("execute_cb done!")
        
if __name__ == '__main__':
    try:
        print("Starting node...")
        rospy.init_node('rss_ompl_actionNode')
        print("Node is now set up!")
        print("Starting server (to be able to give results)...")
        server = RSSOMPLAction(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        print("All done! :)")

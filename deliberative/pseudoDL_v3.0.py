# Copyright 2017-2018 by University of Cincinnati
# Copyright 2014-2016 by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite
"""
=== Summary of contents ===
"""
from time import sleep

# Note that the following may only work if you run this python script from the directory in which it resides...
#
# This is done so you can use the import command on other/separate modules (this is adding it to the beginning like it should've automagically done for you); remember to create an empty __init__.py in the local directory for this to load properly
import os
import sys # for sys.exit() and sys.path.append()
sys.path.append(os.getcwd()) # modify sys.path to include current directory
sys.path.append(os.getcwd() + '/../common') # modify sys.path to include ../common directory
sys.path.append(os.getcwd() + '/../habitual') # modify sys.path to include ../habitual directory (just in case need to use functions from hlFunctions.py)

from waypointFunctions import readWaypointsFromFile
import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
import ws4pyFunctions as wsF
import rosConnectWrapper as rC
import rosdataFunctions as rdF
import fileFunctions as fF

from defaultValuesDL import (menuDL,constRuntimeParamsDL)
#import deliberativeFunctions as dF
import some_constraintFunctions as s_cF
import solverCalls as sC

class DeliberativeLayer(object):
    #
    # general layer functions defined
    #
    
    def __init__(self,argvdefaults,argv): # __init__(self,arg):
        #super(DeliberativeLayer,self).__init__(arg) # super inheritance
        [echo_input,
         self.connection] = fF.readArgvpieces(argv,argvdefaults)
        echo_input = int(echo_input) # in case overwrote int with string
        
        print("Force wait until rosbridge interface and parameter server come online...")
        testReturn = wsF.generalRecvParamServer(self.connection,'/',['rosdistro'],'block',1.0) # wait 1.0sec before recheck
        print("Parameter server is up and responding!")
        
        #[robotType,simType] = wsF.generalRecvParamServer(connection,'/globals/',['robotType','simType'],'block')
        #[stepTime] = constRuntimeParamsDL(robotType,simType)
        Flag_true_from_some_flag_dict = {1: 'yes', 0: None}
        [self.some_flag] = wsF.generalRecvParamServer(self.connection,"/globals/",['some_flag'],'block')
        self.Flag_true = Flag_true_from_some_flag_dict[self.some_flag] # initialize; note that Flag_true = None should stay constant for some_flag=0
        
        [self.waypointFile,self.waypointsOnly] = menuDL(echo_input)
        
        [self.growlen,[self.timetry,self.retries],
         [self.derr,self.mult],self.passthrough,
         self.solverUse,self.mapIsRasterNotPolygons,self.plannerstr,
         [self.intmap,self.rParams,self.rover],
         [self.polytope_list,self.polyunits,self.polydim],
         self.OMPLsolverData,
         syspathstrToAppend] = sC.setupSolverParams(connection=self.connection,globallayertopic="/globals/deliberative/",raster_params=None) # raster_params=[[rwidth,rheight,thresh,self.growlen],mapname,[self.timetry,self.retries]]
        sys.path.append(syspathstrToAppend)
        
        self.someCoeff = 20
        self.relaxBy = 2
        
        self.t = 0; self.t0 = None
        self.loc = [0]*3 ; self.loc0 = [None]*3 # [x,y,z] ; [x0,y0,z0]
        self.ang = [0]*3 ; self.ang0 = [None]*3 # [r,p,h] ; [r0,p0,h0]
        self.quat = [1,0,0,0] ; self.quat0 = [None]*4 # [qx,qy,qz,qw] ; [qx0,qy0,qz0,qw0]
        
        self.new_cur_pos = None

        #self.simSteps = 1;
        #stepTime defined above
        #self.tinc = stepTime
        self.seq=0
        self.current_waypoints = []
        #self.current_waypoints = None
        self.new_current_waypoints = []
        self.risk = []
        self.riskHold = []

        self.pts_calcd = 1 # for saving path-calculation runs
        self.path_list = []
        self.pathpoint_on = 0

        #self.opstatus = None # force it to start as not defined
        self.waypoint_on = -1 # force it to get waypoints at beginning if needed
        self.hlstatus = None #; self.checkthis = True # force it to start sending waypoints at beginning if from file, HL should already be waiting (w/o initial HL status send)
        self.hlwaypoints = None
        self.hlseq = None
        self.hlwayseq = None
        self.some_constraint_request = None
        self.time_request = None
        self.actuator_request = None
        
        self.stateDL = 'start'; self.opmsgreceived = False; self.hlmsgreceived = False
        
        self.senddlstatusWhere = {100: 'senddlstatusToHL', # we are good so far
                                  200: 'senddlstatusToOp', # we found a problem that requires a fail-up
                                  900: 'senddlstatusToOp'} # we found a problem that requires a fail-up
        self.dlstatus = 100 # start assuming things are okay :)
        self.handling = None
        self.handlingHold = None
        
    def bringupRSEconnections(self):
        #
        # receive data request to estimator (keeps number of opened channels from estimator down and network traffic low)
        #
        [stateDataEstOrReal,estChannels] = wsF.generalRecvParamServer(self.connection,'/globals/deliberative/',['stateDataEstOrReal','estChannels'],'block')
        self.estChannels = estChannels

        self.ws_runtype_in = rC.RosMsg('ws4py', self.connection, 'sub', '/rse/runtype', 'std_msgs/Int8', ws4pyROS.unpack_runtype)
        self.ws_hlstatus_in = rC.RosMsg('ws4py', self.connection, 'sub', '/habitual/HL_to_DL', 'rss_msgs/HLtoDL', ws4pyROS.unpack_HLtoDL)
        if (self.some_flag == 0): # (flag_sent == 'no'):
            self.ws_DLtoHLmsg_out = rC.RosMsg('ws4py', self.connection, 'pub', '/deliberative/DL_to_HL', 'rss_msgs/DLtoHLnoFlag', ws4pyROS.pack_DLtoHLnoFlag_ignoreFlag) # for consistency (will ignore some_flag portion of message)
            self.ws_DLtoOpmsg_out = rC.RosMsg('ws4py', self.connection, 'pub', '/deliberative/DL_to_Op', 'rss_msgs/DLtoOpnoFlag', ws4pyROS.pack_DLtoOpnoFLag_ignoreFlag) # for consistency (will ignore some_flag portion of message)
        else: #if (self.some_flag == 1): # (flag_sent == 'yes'):
            self.ws_DLtoHLmsg_out = rC.RosMsg('ws4py', self.connection, 'pub', '/deliberative/DL_to_HL', 'rss_msgs/DLtoHLwithFlag', ws4pyROS.pack_DLtoHLwithFlag)
            self.ws_DLtoOpmsg_out = rC.RosMsg('ws4py', self.connection, 'pub', '/deliberative/DL_to_Op', 'rss_msgs/DLtoOpwithFlag', ws4pyROS.pack_DLtoOpwithFlag)
            #self.some_flag = 0 # set to never use Flag for now
        if (estChannels == 'combined'): # estimator will supply with real or est automatically
            self.ws_est_in = rC.RosMsg('ws4py', self.connection, 'sub', '/estimator/Est_to_DL', 'rss_msgs/EstToDL', ws4pyROS.unpack_esttoDL)
        else: #if (estChannels == 'manysingles'):
            self.ws_pose = rC.RosMsg('ws4py', self.connection, 'sub', '/estimator/pose'+stateDataEstOrReal, 'nav_msgs/Odometry', ws4pyROS.unpack_pose) # /estimator/poseReal or poseEst
            
        if (self.waypointFile is not None): # waypoints from file
            [waypoints_all, units, dim, commentstr] = readWaypointsFromFile(self.waypointFile)
            self.current_waypoints = waypoints_all
        else: # waypoints from ROS node
            if (self.waypointsOnly == True):
                #self.ws_wayptsFromOperator = rC.RosMsg('ws4py', self.connection, 'sub', '/operator/waypoint_list', 'nav_msgs/Path', ws4pyROS.unpack_waypoints)
                self.ws_OptoDL_in = rC.RosMsg('ws4py', self.connection, 'sub', '/operator/waypoint_list', 'nav_msgs/Path', ws4pyROS.unpack_waypoints_ignorerest) # for consistency
            else: # if(waypointsOnly == False):
                self.ws_OptoDL_in = rC.RosMsg('ws4py', self.connection, 'sub', '/operator/Op_to_DL', 'rss_msgs/OptoDL', ws4pyROS.unpack_OptoDL)

        # first waypoint should be on the parameter server
        print("Reading initial position from parameter server, please wait...")
        self.start = wsF.receiveInitPositionParamServer(self.connection)
        self.loc[0:2] = self.start[0:2] # [x,y,z]
        print("All ws4py connections open, initial position received...")
        
    def run(self):
        while (1):
            self.runOnce()

    def runOnce(self):
        self.runStateMachine()
        self.checkForMessages()
        # sleep(sleeptime) # try not to send commands so fast that you overwhelm the system...
        sleep(0.02)
        self.checkForStopFlag()
    
    def runStateMachine(self):
        #
        # interrupt-handler state machine (/ state machine override?)
        #
        #if (self.stateDL == 'checkForOpMessage'):
        if (self.waypointFile is None): # always handle Op message every time through
            if (self.opmsgreceived == False):
                if not(len(self.current_waypoints) > 0): # if we have no waypoints on the queue...
                    print("Still asleep / waiting for new waypoints from operator...")
                    sleep(1) # waiting for response from Operator (for the first time?)
            else: #if (self.opmsgreceived == True):
                self.opmsgreceived = False # set up for next time
                if (len(self.new_current_waypoints) > 0): # we had some change in the waypoints received, so...
                    print("Operator->DL: Waypoints received: %r" % self.new_current_waypoints)
                    print("risk level allowed: %s" % str(self.riskHold))
                    print("DL: waypoint list now: %r" % self.current_waypoints)
                    print("handling = %d" % self.handling)
                    if (self.handling == 0) or (self.handling == 2):
                        [self.seq] = senddlstatusToHL(self.ws_DLtoHLmsg_out,100,[],self.seq,100,'yes') # (re)planning will take awhile, so tell HL to stop for a second in the meantime # dlstatus=100,curwaypt=[],seq=self.seq,someConstraint=100,Flag_true='yes'
                        self.waypoint_on = -1; self.stateDL = 'handleWaypoints'
                    else: #if (handling == 1): # added to the end of stack, continue as usual (don't interrupt current process)
                        #self.waypoint_on = self.waypoint_on
                        #self.stateDL = self.stateDL
                        if (len(self.new_current_waypoints) == len(self.current_waypoints)): # then we didn't have any waypoints on the list before, so treat like a 0 or 2 case!
                            [self.seq] = senddlstatusToHL(self.ws_DLtoHLmsg_out,100,[],self.seq,100,'yes') # (re)planning will take awhile, so tell HL to stop for a second in the meantime # dlstatus=100,curwaypt=[],seq=self.seq,someConstraint=100,Flag_true='yes'
                            self.waypoint_on = -1; self.stateDL = 'handleWaypoints'
                else: # we have no waypoints to complete, so request Op again?
                    [self.seq] = senddlstatusToHL(self.ws_DLtoHLmsg_out,100,[],self.seq,100,'yes') # new waypoints may be awhile in coming, so tell HL to stop for a second in the meantime # dlstatus=100,curwaypt=[],seq=self.seq,someConstraint=100,Flag_true='yes'
                    #print("DL->Operator: Requesting next waypoint(s)")
                    self.dlstatus = 900; self.stateDL = 'senddlstatusToOp'
        #
        # state machine that controls transitions between internal states
        # (shows transition behaviors explicitly)
        #
        if (self.stateDL == 'start'):
            if (self.waypointFile is not None):
                if (len(self.current_waypoints) > 0):
                    print("File->DL: Waypoints received: %r" % self.current_waypoints)
                    self.waypoint_on = -1; self.stateDL = 'handleWaypoints'
                else:
                    print("Done and shutting down.")
                    self.dlstatus = 200; self.stateDL = 'senddlstatusToHL'
            else:
                self.current_waypoints = []; self.new_current_waypoints = []
                self.risk = []; self.riskHold = []
                self.stateDL = 'checkForOpMessage'
        elif (self.stateDL == 'handleWaypoints'):
            if (self.passthrough == 'yes'):
                self.waypoint_on += 1
                if (len(self.current_waypoints) > self.waypoint_on):
                    print("DL: Next waypoint being used: %s" % str(self.current_waypoints[self.waypoint_on]))
                    self.path_list = [self.current_waypoints[self.waypoint_on]]
                    self.pathpoint_on = 0
                    self.stateDL = 'computeSomeConstraint'
                else: # we have no waypoints to complete
                    if (self.waypointFile is not None): # since from file, we are done
                        print("Done and shutting down.")
                        self.dlstatus = 200; self.stateDL = 'senddlstatusToHL'
                    else: # so request Op again?
                        #print("DL->Operator: Requesting next waypoint(s)")
                        self.dlstatus = 900; self.stateDL = 'senddlstatusToOp'
            else: #if (self.passthrough == 'no'): # then need to compute waypoints in a similar manner as we do at the HL level
                if (len(self.path_list) > 0):
                    self.pathpoint_on += 1
                if (len(self.path_list) > self.pathpoint_on):
                    print("DL: Next waypoint being used: %s" % str(self.path_list[self.pathpoint_on]))
                    #self.path_list = self.current_waypoints[self.waypoint_on]
                    self.stateDL = 'computeSomeConstraint'
                else: # we have no waypoints to complete in path_list, so we need to recompute a new sublist using the next point in self.current_waypoints
                    # self.waypoint_on += 1 # handled/incremented inside handleWaypoints()
                    #print("DL: Next waypoint being used: %s" % str(self.current_waypoints[self.waypoint_on]))
                    self.path_list = [] # shouldn't hurt anything to init off the bat
                    self.pathpoint_on = 0 # shouldn't hurt anything to init off the bat,this far up
                    from hlFunctions import handleWaypoints
                    #[self.waypoint_on,self.hlstatus,
                    # self.pts_calcd,
                    # self.path_list] = handleWaypoints(self.connection,'HL',self.start,self.rParams,self.growlen,
                    #                                   self.waypoint_on,self.current_waypoints,
                    #                                   self.pts_calcd,self.derr,self.mult,self.passthrough,
                    #                                   self.solverUse,self.plannerstr,
                    #                                   self.OMPLsolverData,
                    #                                   self.retries)
                    #if (len(self.current_waypoints) <= self.waypoint_on): # then we computed nothing / we have read everything in the current_waypoints list
                    #    self.stateHL = 'sendhlstatusToDL'
                    #elif (self.hlstatus == 100): # we are good so far
                    #    self.stateHL = 'checkActuatorStatus'
                    #else: # we found a problem that requires a fail-up
                    #    self.stateHL = 'sendhlstatusToDL'
                    start = self.loc # get current location
                    [self.waypoint_on,self.dlstatus,
                     self.pts_calcd,
                     self.path_list] = handleWaypoints(self.connection,'DL',self.start,self.rParams,self.growlen,
                                                       self.waypoint_on,self.current_waypoints,
                                                       self.pts_calcd,self.derr,self.mult,self.passthrough,
                                                       self.solverUse,self.plannerstr,
                                                       self.OMPLsolverData,
                                                       self.retries)
                    if (len(self.current_waypoints) <= self.waypoint_on): # then we computed nothing / we have read everything in the current_waypoints list
                        # we have no waypoints to complete
                        print("DL: Finished all waypoints in list.")
                        if (self.waypointFile is not None): # since from file, we are done
                            print("DL: Done and shutting down.")
                            self.dlstatus = 200; self.stateDL = 'senddlstatusToHL'
                        else: # so request Op again?
                            #print("DL->Operator: Requesting next waypoint(s)")
                            self.dlstatus = 900; self.stateDL = 'senddlstatusToOp'
                    elif (self.dlstatus == 100): # we are good so far
                        #print("DL: Next waypoint being used: %s" % str(self.current_waypoints[self.waypoint_on]))
                        print("DL: Waypoints calculated: %s" % str(self.path_list))
                        #self.pathpoint_on = 0
                        print("DL: Next waypoint being used: %s" % str(self.path_list[self.pathpoint_on]))
                        self.stateDL = 'computeSomeConstraint'
                    else: # we found a problem that requires a fail-up
                        print("DL: Unresolvable error without operator intervention.")
                        print("DL: Stopping.")
                        if (self.waypointFile is not None): # since from file, we are done
                            self.stateDL = 'senddlstatusToHL' #; self.dlstatus = 200
                        else: # so tell Op a problem occurred
                            self.stateDL = 'senddlstatusToOp' #; self.dlstatus = 200
        #elif (self.stateDL == 'checkRiskLevel'):
        #    [???] = checkRiskLevel(self.current_waypoints,self.waypoint_on,self.loc,self.risk,???)
        #    self.stateDL = 'computeSomeConstraint'
        #    if (self.dlstatus != 100): # we found a problem that requires a fail-up
        #        self.stateDL = 'senddlstatusToOp'
        elif (self.stateDL == 'computeSomeConstraint'): # is still / currently sending waypoints one at a time from DL to HL
            #[self.curwaypt,self.someConstraint,self.dlstatus] = computeSomeConstraint(self.current_waypoints,self.waypoint_on,self.loc,self.some_constraintCoeff)
            [self.curwaypt,self.someConstraint,self.dlstatus] = computeSomeConstraint(self.path_list,self.pathpoint_on,self.loc,self.some_constraintCoeff)
            self.stateDL = self.senddlstatusWhere[self.dlstatus]
        elif (self.stateDL == 'relaxSomeConstraint'):
            [self.someConstraint,self.some_constraintCoeff,self.dlstatus] = relaxSomeConstraint(self.someConstraint,self.some_constraintCoeff,self.relaxBy)
            self.stateDL = self.senddlstatusWhere[self.dlstatus]
        elif (self.stateDL == 'relaxTimeConstraint'):
            [self.curwaypt,self.dlstatus] = relaxTimeConstraint(self.curwaypt)
            self.stateDL = self.senddlstatusWhere[self.dlstatus]
        elif (self.stateDL == 'senddlstatusToHL'):
            [self.seq] = senddlstatusToHL(self.ws_DLtoHLmsg_out,self.dlstatus,[self.curwaypt],self.seq,self.someConstraint,self.Flag_true)
            self.stateDL = 'checkForHLMessage'
        elif (self.stateDL == 'checkForHLMessage'):
            [[self.hlstatus,self.hlwaypoints,self.hlseq,
              self.hlwayseq,self.some_constraint_request,
              self.time_request,self.actuator_request],
             self.hlmsgreceived] = checkForHLMessage(self.ws_hlstatus_in,
                                                     [self.hlstatus,self.hlwaypoints,self.hlseq,
                                                     self.hlwayseq,self.some_constraint_request,
                                                     self.time_request,self.actuator_request])
            if (self.hlmsgreceived == False):
                sleep(1) # waiting for response from HL (for the first time?)
            else: #if (hlmsgreceived == True):
                if (self.hlstatus == 200):
                    print("HL->DL: Received hlstatus of 200, stopping...")
                    #break
                    self.shutdown()
                self.stateDL = 'handleHLstatus'
        elif (self.stateDL == 'handleHLstatus'): # if we just completed this, then...
            [hlrequest,self.dlstatus] = handleHLstatus(self.hlstatus,self.dlstatus)
            hlrequestTostateDLdict = {'next waypoint':    'handleWaypoints',
                                 'fail to operator': 'senddlstatusToOp',
                                 'relax some_constraint':      'relaxSomeConstraint',
                                 'relax time':       'relaxTimeConstraint'}
            self.stateDL = hlrequestTostateDLdict[hlrequest]
        elif (self.stateDL == 'senddlstatusToOp'): # if we just completed this, then...
            [self.seq] = senddlstatusToOp(self.ws_DLtoOpmsg_out,self.dlstatus,self.current_waypoints,self.seq,self.Flag_true)
            self.current_waypoints = []; self.new_current_waypoints = []
            self.risk = []; self.riskHold = []
            self.stateDL = 'checkForOpMessage'
        
    def checkForMessages(self):
        # always check for Op message every time through (if not reading points from waypoint file)
        if (self.waypointFile is None):
            [self.opmsgreceived,
             [self.new_current_waypoints,
              self.riskHold,self.handlingHold],
             [self.current_waypoints,
              self.risk,
              self.handling]] = checkForOpMessage(self.ws_OptoDL_in,
                                                  [self.new_current_waypoints,
                                                   self.riskHold,self.handlingHold],
                                                  [self.current_waypoints,
                                                   self.risk,
                                                   self.handling],
                                                  self.waypoint_on)#,self.waypointsOnly):
        
        # grab current state from ROSARIA/ROS ==every time==
        if (self.estChannels == 'combined'): # estimator will supply with real or est automatically
            [self.new_cur_pos] = self.ws_est_in.copy_and_clear_received([self.new_cur_pos])
        else: #if (estChannels == 'manysingles'):
            self.new_cur_pos = self.ws_pose.copy_and_clear_received(self.new_cur_pos)
        [self.t0,self.t,self.loc0,self.loc,self.quat0,self.quat,self.ang0,self.ang] = rdF.poseExtract(self.new_cur_pos,[self.t0,self.t,self.loc0,self.loc,self.quat0,self.quat,self.ang0,self.ang])
            
    def checkForStopFlag(self):
        if (self.dlstatus == 200): # then we need to stop
            if (self.waypointFile is not None): # not receiving another set of waypoints, was hardcoded internally
                print("dlstatus = %d, stopping..." % self.dlstatus)
                self.shutdown()
        # note: # if sim tells us to stop for whatever reason, then stop
        runtype_Stop = self.ws_runtype_in.copy_and_clear_received()
        if ((runtype_Stop is not None) and (runtype_Stop == 0)):
            print("Got a runtype = %d, stopping..." % runtype_Stop)
            self.shutdown()
        
    def shutdown(self):
        #
        # tell everything else to shutdown as well (if we finish first for some reason)
        #
        rC.sendRunTypeValueAndShutdownWs4py(0,self.connection,ws_runtype_in=self.ws_runtype_in,ws_runtype_out=None) # runtype = 0 for shutdown remote sim. + habitual layer

        #print("Shutting down simulator and exiting...")

        self.ws_hlstatus_in.closeNow()
        self.ws_DLtoHLmsg_out.closeNow()
        self.ws_DLtoOpmsg_out.closeNow()
        
        if (self.estChannels == 'combined'): # estimator will supply with real or est automatically
            self.ws_est_in.closeNow()
        else: #if (estChannels == 'manysingles'):
            self.ws_pose.closeNow()
        
        if (self.waypointFile is None):
            self.ws_OptoDL_in.closeNow()

        sleep(1)

        print("All done! :)")
        
        sys.exit(0)

#
# layer behavioral actions defined
#

def checkForHLMessage(ws_hlstatus_in,dataUpdate):
    [hlstatus,hlwaypoints,hlseq,hlwayseq,some_constraint_request,time_request,actuator_request] = dataUpdate
    hlmsgreceived = False
    hlhold = ws_hlstatus_in.copy_and_clear_received()
    if (hlhold is not None):
        [hlstatus,hlwaypoints,blah3,blah4,blah5,blah6,blah7] = hlhold
        if ((hlstatus == 100) and (len(hlwaypoints) == 0)): # don't deal with msg if 100 didn't actually handle waypoints (not a ping from an empty list [])
            hlmsgreceived = False
        else:
            [blah1,blah2,hlseq,hlwayseq,some_constraint_request,time_request,actuator_request] = hlhold
            #checkthis = hlstatus != None # should be True if reach this point
            print("HL->DL: hlstatus=%d" % hlstatus)
            hlmsgreceived = True
    return [[hlstatus,hlwaypoints,hlseq,hlwayseq,some_constraint_request,time_request,actuator_request],hlmsgreceived]

def computeSomeConstraint(current_waypoints,waypoint_on,loc,some_constraintCoeff):
    curwaypt = list(current_waypoints[waypoint_on])
    someConstraint = s_cF.calculateSomeConstraint(curwaypt,loc[0],loc[1],some_constraintCoeff)
    print("DL: someConstraint = %r" % someConstraint)
    dlstatus = 100
    return [curwaypt,someConstraint,dlstatus]

def relaxSomeConstraint(someConstraint,some_constraintCoeff,relaxBy):
    print("DL: (original) someConstraint = %r" % someConstraint)
    [someConstraint,some_constraintCoeff] = s_cF.relaxSomeConstraint(someConstraint,some_constraintCoeff,relaxBy)
    print("DL: (relaxed) someConstraint = %r" % someConstraint)
    dlstatus = 100
    return [someConstraint,some_constraintCoeff,dlstatus]
    
def relaxTimeConstraint(curwaypt):
    print("DL: (original) curwaypt = %r" % curwaypt)
    curwaypt[0] = curwaypt[0] - 0.5 # relax time constraint (change point by -0.5 in x and y?)
    curwaypt[1] = curwaypt[1] - 0.5 # relax time constraint (change point by -0.5 in x and y?)
    print("DL: (relaxed time) curwaypt = %r" % curwaypt)
    dlstatus = 100
    return [curwaypt,dlstatus]

def checkForOpMessage(ws_OptoDL_in,oldop,curop,waypoint_on): #,waypointsOnly):
    [new_current_waypoints,riskHold,handlingHold] = oldop
    [current_waypoints,risk,handling] = curop
    opmsgreceived = False # added line (reset this value before begin)
    ophold = ws_OptoDL_in.copy_and_clear_received()
    if (ophold is not None):
        print("DL: Message received from operator, parsing...")
        [blah1,blah2,handlingHold] = ophold
        if (handlingHold >= 3) or (handlingHold < 0):
            print("Message received, but bad handling information (%s). Waiting for 'working' message." % str(handlingHold))
        else: # if (handlingHold < 3) and (handlingHold >= 0):
            handling = handlingHold
            opmsgreceived = True
            [new_current_waypoints,riskHold,blah3] = ophold
            #if (waypointsOnly == True): # if waypointsOnly, then assume...
                #riskHold = 100 # risk is not a problem (full 100%) # set because this is given back as a 'None'
                #handlingHold = 0 # receive is always an override situation (handling) # set because this is given back as a 'None'
            if (handlingHold == 0): # drop prev. waypoints/commands and run this now/replace with this
                current_waypoints = new_current_waypoints
                risk = [riskHold]*len(new_current_waypoints)
            elif (handlingHold == 1): # add this to the end of stack
                current_waypoints += new_current_waypoints
                risk += [riskHold]*len(new_current_waypoints)
            elif (handlingHold == 2): # pause current and add this to the start of the stack
                if (waypoint_on > -1):
                    holdcurrentptslist = list(current_waypoints[waypoint_on::])
                else:
                    holdcurrentptslist = list(current_waypoints)
                holdrisklist = list(risk)
                current_waypoints = new_current_waypoints + holdcurrentptslist
                risk = [riskHold]*len(new_current_waypoints) + holdrisklist
    return [opmsgreceived,[new_current_waypoints,riskHold,handlingHold],[current_waypoints,risk,handling]]

def handleHLstatus(hlstatus,dlstatus):
    """
    assumption is that new/next waypoint will be sent for all messages except 000
    *** NOTE: for now, assume that we only get one msg per cycle ***
    """
    if (hlstatus == 100): # waypoint achieved
        hlrequest = 'next waypoint'
    elif (hlstatus == 200 or hlstatus == 211 or hlstatus == 221 or hlstatus == 232): # waypoint not achieved, unknown error; some_constraint requirement has failed; time requirement has failed; waypoint not achieved, actuator failure detected
        hlrequest = 'fail to operator' ; dlstatus = 200
    elif (hlstatus == 210 or hlstatus == 231): # waypoint not achieved, some_constraint requirement will fail; waypoint not achieved, actuator degradation detected
        if (len(current_waypoints) > waypoint_on): # more waypoints
            hlrequest = 'relax some_constraint'
        else: # should never get this far(?)
            hlrequest = 'fail to operator' ; dlstatus = 200
            print("DL: Cannot relax some_constraint; failing up to operator.")
    elif (hlstatus == 220): # waypoint not achieved, time requirement will fail # *** THIS IS BUGGY AND WILL NOT WORK AS IT STANDS!! ***
        if (len(current_waypoints) > waypoint_on): # more waypoints
            hlrequest = 'relax time'
        else: # should never get this far(?)
            hlrequest = 'fail to operator' ; dlstatus = 200
            print("DL: Cannot relax time constraint; failing up to operator.")
    return [hlrequest,dlstatus]

def senddlstatusToHL(ws_DLtoHLmsg_out,dlstatus,curwaypt,seq,someConstraint,Flag_true):
    if (dlstatus == 100): # waypoint sent (to HL)
        print("DL->HL: sending next waypoint (%s), someConstraint (%f), Flag_true=%r" % (str(curwaypt),someConstraint,Flag_true)); sendpt = curwaypt#; sendSomeConstr = someConstraint
    elif (dlstatus == 200): # unknown error or shutdown
        print("DL->HL: shutting down..."); sendpt = [(None,None)]#; sendSomeConstr = 0
    ws_DLtoHLmsg_out.send_pieces([dlstatus,[sendpt, 1],seq,someConstraint,0,Flag_true]); seq += 1 # [waypoint, waytype=1], and some_constraint, and holding=0 of "follow immediately"
    return [seq]

def senddlstatusToOp(ws_DLtoOpmsg_out,dlstatus,current_waypoints,seq,Flag_true):
    if (dlstatus == 900): # requesting more waypoints from operator
        print("DL->Operator: Requesting next waypoint(s)"); sendpt = current_waypoints
    elif (dlstatus == 200): # unknown error or shutdown
        print("DL->Operator: Failing up to operator"); sendpt = [(None,None)]
    ws_DLtoOpmsg_out.send_pieces([dlstatus,[sendpt, 1],seq,0,Flag_true]); seq += 1
    return [seq]

if __name__ == '__main__':
    
    try:
        print("sys.argv = " + str(sys.argv))
        # python pseudoDL.py echo_input connection ...
        argvdefaults = [1,
                        "ws://localhost:9090/"] # default connection valid because starting rosbridge from this file :)
        
        DL3 = DeliberativeLayer(argvdefaults,sys.argv)
        DL3.bringupRSEconnections()
        DL3.run()

        sys.exit(0)

    except KeyboardInterrupt:
        try:
            DL3.shutdown()
            sleep(1)
        except NameError:
            pass

        sys.exit(0)

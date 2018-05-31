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

import rastermapFunctions as rmF # in ../common
#from waypointFunctions import readWaypointsFromFile
#import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
import ws4pyFunctions as wsF
#import rosConnectWrapper as rC
#import rosdataFunctions as rdF
#import fileFunctions as fF

def setupSolverParams(connection,globallayertopic = "/globals/habitual/",raster_params=None): # raster_params = [[rwidth,rheight,thresh,self.growlen],mapname,[self.timetry,self.retries]]

    if (raster_params is None): # for now, set default manually and assume p3dx run always:
        [[rwidth,rheight,thresh,self_growlen],mapname,
         [self_timetry,self_retries]] = [[100,100,None,0],None,[30,3]] # read data from map server
    else:
        [[rwidth,rheight,thresh,self_growlen],mapname,
         [self_timetry,self_retries]] = raster_params
    
    [self_derr,self_mult] = wsF.generalRecvParamServer(connection,"/globals/constRuntimeParams/",['derr','mult'],'block')
    
    [self_passthrough] = wsF.generalRecvParamServer(connection,globallayertopic,['passthrough'],'block')
    
    [self_solverUse] = wsF.generalRecvParamServer(connection,globallayertopic,['solverUse'],'block')
    self_plannerstr = None
    solverToMapTypedict = {'OMPL': True}
    self_mapIsRasterNotPolygons = None
    if (self_passthrough == 'no'): # then we are computing paths and need these things
        self_mapIsRasterNotPolygons = solverToMapTypedict[self_solverUse]
        if (self_solverUse == 'OMPL'):
            [self_plannerstr] = wsF.generalRecvParamServer(connection,globallayertopic,['plannerstr'],'nonblock')
            if (self_plannerstr is None): # not received from server
                self_plannerstr = 'RRTstar' # default planner to use for OMPL
        #else: plannerstr stays = None
        
        #if (self.mapIsRasterNotPolygons == True):
        # set up raster map
        [self_intmap,self_rParams,self_rover] = rmF.rasterMapSetup(mapname,connection,thresh,rwidth,rheight,self_growlen) # requires: import polytope as pc
        # set up polygon/polytope obstacle info (for collision checks in a non-raster map)
        #self.polytope_list = []
        #[self.polyunits,self.polydim] = [None,None]
        #else: #if (self.mapIsRasterNotPolygons == False):
        #self.intmap = None; self.rParams = None; self.rover = None
        # set up polygon/polytope obstacle info (for collision checks in a non-raster map)
        [self_polytope_list,self_polyunits,self_polydim] = wsF.generalRecvParamServer(connection,"/globals/obstacles/",['polytope_list','units','dim'])
        self_OMPLsolverData = [self_intmap,self_rover,self_timetry,self_retries] # general stuff requires intmap currently (graphical map file writeout)
        if (self_solverUse == 'OMPL'):
            # set up path planner: RasterMap class (that uses OMPL 1.1.1) requires map/environment and 'rover' information given above
            #self_OMPLsolverData = [self.intmap,self.rover,self.timetry,self.retries]
            self_OTHERsolverData = []
            #sys.path.append(os.getcwd() + '/../algorithms/rss_ompl_wrapper/scripts') # modify sys.path to include ../algorithms/rss_ompl_wrapper directory
            syspathstrToAppend = os.getcwd() + '/../algorithms/rss_ompl_wrapper/scripts' # modify sys.path to include ../algorithms/rss_ompl_wrapper directory
        #elif (self_solverUse == 'OTHER_solver'):
            #self_OMPLsolverData = [None]*4
            ##self_OTHERsolverData = [None]*4
            ##...
            ##...
            #pass
        else:
            print("Warning: solverUse not recognized: %s" % self.solverUse)
        
    else:
        self_intmap = None; self_rParams = None; self_rover = None
        self_OMPLsolverData = [None]*4
        self_polytope_list = []
        [self_polyunits,self_polydim] = [None,None]
        syspathstrToAppend = os.getcwd() + '/../common' # should hurt nothing to (re)include common directory
        pass; # for now, nothing 'extra' needs to be done if just passing waypoints through

    return [self_growlen,[self_timetry,self_retries],
            [self_derr,self_mult],self_passthrough,
            self_solverUse,self_mapIsRasterNotPolygons,self_plannerstr,
            [self_intmap,self_rParams,self_rover],
            [self_polytope_list,self_polyunits,self_polydim],
            self_OMPLsolverData,
            syspathstrToAppend]

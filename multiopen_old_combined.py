#!/usr/bin/env python
# Copyright 2017-2018 by University of Cincinnati
# Copyright 2015-2016 by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite
"""
=== Summary of contents ===

This script runs various sets of windows+tabs together as-necessary (according to given runScriptType and other input parameters).

Takes up to seven (7) command-line arguments:
-- runScriptType is a string, one of: {'MobileSim','p3dxgazebo'} (default="MobileSim")
-- mapFrom is a string (default='default_polytopes.txt')
-- startPtFile is a string (default="default_startpt.txt")
-- RLcall is the reflexive layer python file to run (default="ReflexiveLayer_v2.2.py")
-- HLcall is the habitual layer python file to run (default="HabitualLayer_v2.2.py")
-- DLcall is the deliberative layer python file to run (default="pseudoDL.py")
-- connection is a string (default="ws://localhost:9090/")
Can set any/all of these to 'def' for default params, or leave blank at the end, e.g.:
$ python multiopen_old_combined.py
$ ./multiopen_old_combined.py MobileSim def def def
$ ./multiopen_old_combined.py MobileSim default_polytopes.txt default_startpt.txt ReflexiveLayer_v3.0.py HabitualLayer_v3.0.py pseudoDL.py ws://localhost:9090/

All runScriptType's run: catkin_make, csvmapFromPolytopes.py, roscore, rosbridge_server, PID gains publisher, startpt param push.
robotType='MobileSim' also runs: MobileSim, rosaria

   runScriptType      robotType       mapFrom                 startPtFile           RLcall                   HLcall                  DLcall             connection
-- 'MobileSim'   runs MobileSim with: default_polytopes.txt , default_startpt.txt , ReflexiveLayer_v3.0.py , HabitualLayer_v3.0.py , pseudoDL_v3.0.py , ws://localhost:9090/
-- 'p3dxgazebo'      is the same as 'MobileSim' except for simType='gazebo' and heightMap = ''

Reads start point from file (startPtFile) and pushes it to ROS parameter server:
-- starting location (default=[8.0, -8.0])

currently assumes directory structure to look like:
~/
  catkin_ws
      src
          rosaria
          rss_work
              maps
              p3dx_mover
              PioneerModel
              rss_git
                  3rdparty
                  algorithms
                  common
                  deliberative
                      PathPlanner
                  estimator
                  habitual
                  initialize-params
                  reflexive
                  rss_maps
                  rss_msgs
                  rss_waypoints
                  trackem
                      contrib
                          trackem_ros
                              nodes
"""

from time import sleep
import subprocess

# Note that the following may only work if you run this python script from the directory in which it resides...
#
# This is done so you can use the import command on other/separate modules (this is adding it to the beginning like it should've automagically done for you); remember to create an empty __init__.py in the local directory for this to load properly
import os
import sys # for sys.exit() and sys.path.append()
#sys.path.append(os.getcwd()) # modify sys.path to include current directory
sys.path.append(os.getcwd() + '/./common') # modify sys.path to include ../common directory
import fileFunctions as fF

# (note: might need to 'chmod +x' this file before running it)

# subprocess work expanded from: https://exyr.org/2011/gnome-terminal-tabs/
#
# multiple lines ('\n') with echo (or printf):
# http://stackoverflow.com/questions/23929235/bash-multi-line-string-with-extra-space
# bash example:
#    text="this is line one
#    this is line two
#    this is line three"
#    echo "$text" > filename
# alt. bash example:
#    printf "%s\n" "this is line "{one,two,three} > filename

# terminal titles: http://askubuntu.com/questions/446041/setting-terminal-tab-titles
#
# NOTE: like this, roscore and roslaunch do not take a title from '-t'
# (the commands at the command line for some reason force it otherwise)
# but the python windows have their titles set just fine! :)
#
# (more info on enviroment variable PS1, if I want to go that route later: )
# http://askubuntu.com/questions/30988/how-do-you-set-the-title-of-the-active-gnome-terminal-from-the-command-line

# -------------------------------------------------------------------- #

def printInstructionsToScreen(simType):
    if (simType == 'MobileSim') or (simType == 'p3dxgazebo'):
        print("Note: start HL, wait for all connections to open, then start DL, in that order:")
        print(" ")
        print("For a 'normal' HL run:")
        print("...in the HL window, type:")
        print("1 -- nominal MobileSim HL run (uses RRT*)")
        print("==alternately==")
        print("For a 'HL passthrough' run:")
        print("...in the HL window, type:")
        print("4 -- mode-1 run with passthrough (doesn't use RRT*)")
        print(" ")
        print("For a 'normal' pseudo-DL run...")
        print("...in the pseudo-DL window, type:")
        print("1 -- MobileSim/p3dx run")

def printInstructionsToScreen_special(runScriptType):
    if (runScriptType == 'EXAMPLE'):
        print(" ")
        print("Wait 'til all connections are open...")
        print("then start DL (recommend: 2), wait 'til all connections are open...")
        print("then open another tab and call:")
        print("$ cd ~/catkin_ws && source devel/setup.bash && catkin_make && cd src/rss_work/rss_git")
        print("then run either:")
        print("$ python operator_waypt_test1b.py")
        print("or")
        print("$ python operator_waypt_test2b.py")
        print(" ")
    if (runScriptType == 'EXAMPLE2'):
        print("Must run EXAMPLE2 separately.")
        print(" ")

# -------------------------------------------------------------------- #

if __name__ == '__main__':
    topdir = '../../..' # where toplevel stuff needs to get to
    cddir = './src/rss_work/rss_git' # where we start

    print("Remember to run this from the top level of your rss_git directory, e.g.:")
    print("$ cd ~/catkin_ws/src/rss_work/rss_git")
    print("$ ./multiopen_old_combined.py runScriptType mapFrom startPtFile RLcall HLcall DLcall connection")
    print("where runScriptType = MobileSim -or- p3dxgazebo")
    print(" ")

    print("Starting multiopen_old_combined.py...")
    try:
        print("sys.argv = " + str(sys.argv))

        # python multiopen_old_combined.py runScriptType mapFrom startPtFile Estcall RLcall HLcall DLcall connection ...
        argvdefaults = ["MobileSim",
                        '../input_files/default_polytopes.txt',
                        "../input_files/default_startpt.txt", # [8.0, -8.0, 0.0]
                        'pseudoEstimator_v3.0.py',
                        'ReflexiveLayer_v3.0.py',
                        'HabitualLayer_v3.0.py',
                        'pseudoDL_v3.0.py',
                        "ws://localhost:9090/"] # default connection valid because starting rosbridge from this file :)
        [runScriptType,
         mapFrom,
         startPtFile,
         Estcall,
         RLcall,
         HLcall,
         DLcall,
         connection] = fF.readArgvpieces(sys.argv,argvdefaults)
        
        #
        # check/change some of your input parameters before proceeding
        #

        if (runScriptType == 'MobileSim') or (runScriptType == 'MobileSimMD'):
            echo_input = 1 # 1 for MobileSim default, 2 for OTHERSIM default
            robotType = 'p3dx'
            simType = 'MobileSim'
        elif (runScriptType == 'p3dxgazebo'):
            echo_input = 1
            robotType = 'p3dx'
            simType = 'gazebo'
        else:
            print('Error: unknown runScriptType. Exiting.')
            sys.exit(0)
        """
        Note: supported robotType: {'p3dx', 'MobileSim'}
        to-be-acceptable robotType = {'turtlebot'}
        Note: supported simType: {'MobileSim','hardware','gazebo'}
        to-be-acceptable simType = {'player/stage','rviz','mapviz'}
        """
        
        some_flag = 1 # 'yes'
        csvparamsFile = '../input_files/default_csvparams.txt'
        mobilesimMap = 'runFunctionsArMap.map'
        gazeboworldMap = '../contrib/p3dx_gazebo_mod/worlds/p3dx_mod.world'
        csvOutFile = 'obstacles.csv'
        boundaryedgesFile = 'None' # will be treated as a None
        linearfile = None ; angularfile = None; maxlinangratesfile = None
        estROStopicsfile = '../input_files/default_estorreal.txt'
        actuatordegrade = 'time' # default, but could be 'None'
        DLsolverUse = 'None'
        DLplannerstr = 'None'
        HLsolverUse = 'OMPL' # solverToMapTypedict = {'OMPL': True, 'other': False}
        HLplannerstr = 'RRTstar'
        DLpassthrough = 'yes' # default = no internal calculations
        HLpassthrough = 'no' # default = internal calculations
        if (runScriptType == 'MobileSimMD'):
            print("runScriptType is '%s'" % runScriptType)
            Estcall = 'pseudoEstimator_v3.0.py 2 %s' % connection # starts it with echo_input = 2, connection = connection
            print("--> overriding Estcall param, running Est script with alternative values = '%s'" % Estcall)            
        
        if ([robotType,simType] == ['p3dx','gazebo']): #(runScriptType == 'p3dxgazebo'):
            mapFrom = mapFrom # '../input_files/default_polytopes.txt'
            #mapFrom = '../input_files/default_heightmap.txt' # heightMap
            #gazeboworldMap = os.getcwd() + '/./contrib/p3dx_gazebo_mod/worlds/p3dx_mod_heightmap.world' # original test location
            gazeboworldMap = os.getcwd() + '/rss_maps/p3dx_polygons.world'
            
        # run catkin_make FIRST in the original terminal window and wait
        # 'til it's complete before doing any else! (otherwise you'll get
        # weird ROS ID error messages from the roscore and rosbridge_server
        # if the messages were changed/updated but not recompiled)
        command = ['bash', '-c', '''
                        cd %(topdir)s
                        source devel/setup.bash && catkin_make
        ''' % locals()]
        subprocess.call(command) # each call originates from original directory
                                 # so "cd" here is "ignored" further down :)

        print(" ")
        print(" ")
        print(" ")

        printInstructionsToScreen(simType)
        printInstructionsToScreen_special(runScriptType)

        #
        # the subprocess call doesn't wait for a return in-between commands
        # or between tab-openings when run locally (it will wait to open
        # new windows until previous window is closed if remote)
        # --> with 'gnome-terminal', everything starts almost right away
        # ...so we need force some wait times:
        #
        MobileSim_offset = 7 # 3 # upped for remote access users, if running this over the network remotely
        
        #
        # init terminal string params for terminal run:
        #
        tabtitle = []
        command = []

        # rosbridge_server should automatically start roscore for us
        tabtitle.extend(['rosbridge_server (status)'])
        command.extend([''' source %(topdir)s/devel/setup.bash
                            roslaunch rosbridge_diff_websocket.launch
        ''' % locals()])
        
        tabtitle.extend(['robotType, simType, some_flag, PID gains, estROStopics, actuatordegrade publisher'])
        command.extend([''' cd initialize-params
                            python robotTypesimTypePIDs_paramserver.py %(connection)s %(robotType)s %(simType)s %(some_flag)d %(linearfile)s %(angularfile)s %(maxlinangratesfile)s %(estROStopicsfile)s %(actuatordegrade)s %(DLsolverUse)s %(DLplannerstr)s %(HLsolverUse)s %(HLplannerstr)s %(DLpassthrough)s %(HLpassthrough)s
        ''' % locals()])

        # run csvmapFromPolytopes.py SECOND to create the .csv file maps that
        # will be used for the HL (rss_maps/obstacles.csv) and RL (MobileSim ArMap)
        # (rss_maps/runFunctionsArMap.map)
        # note: the polytope file (mapFrom) is expected to be found under ./rss_maps
        # note: (currently) you must make sure that the polytopes DL is using matches these
        if (simType == 'MobileSim'):
            tabtitle.extend(['MobileSim (status), after map creation + {startpt, map data} param push'])
            command.extend([''' cd rss_maps
                                python csvmapFromPolytopes.py %(mapFrom)s %(startPtFile)s %(csvparamsFile)s %(mobilesimMap)s %(csvOutFile)s %(boundaryedgesFile)s %(connection)s
                                MobileSim -m %(mobilesimMap)s -r %(robotType)s
            ''' % locals()])
            tabtitle.extend(['rosaria (status)'])
            command.extend([''' source %(topdir)s/devel/setup.bash
                                python test_ws4pyCheck.py %(connection)s
                                sleep %(MobileSim_offset)f
                                MOBILESIM_UP=`ps -A | grep -o MobileSim | wc -l`
                                while [ $MOBILESIM_UP -ne 1 ]
                                do
                                    echo "Waiting on MobileSim to come up..."
                                    sleep 1
                                    MOBILESIM_UP=`ps -A | grep -o MobileSim | wc -l`
                                done
                                echo "MobileSim looks to be up! Running RosAria in 2 seconds..."
                                sleep 2
                                rosrun rosaria RosAria
            ''' % locals()])
        elif (simType == 'gazebo'):
            # $ sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers
            # $ cd ~/catkin_ws/src/rss_work
            # $ git clone https://github.com/SD-Robot-Vision/PioneerModel.git
            # $ git clone https://github.com/SD-Robot-Vision/p3dx_mover.git
            # $ source ../../devel/setup.bash && catkin_make
            
            # this is super-messy for now...
            #from waypointFunctions import readWaypointsFromFileSimple # ../common
            #startpt = readWaypointsFromFileSimple(filename='./input_files/'+startPtFile)
            #startPtFilex = startpt[0][0]
            #startPtFiley = startpt[0][1]
            #startPtFilez = startpt[0][2]
            tabtitle.extend(['map creation + {startpt, map data} param push, then pop gazebo status window'])
            command.extend([''' source %(topdir)s/devel/setup.bash
                                cd rss_maps
                                echo python csvmapFromPolytopes.py %(mapFrom)s %(startPtFile)s %(csvparamsFile)s %(gazeboworldMap)s %(csvOutFile)s %(boundaryedgesFile)s %(connection)s
                                python csvmapFromPolytopes.py %(mapFrom)s %(startPtFile)s %(csvparamsFile)s %(gazeboworldMap)s %(csvOutFile)s %(boundaryedgesFile)s %(connection)s
                                echo python subcall_gazebo.py %(gazeboworldMap)s %(connection)s
                                ./subcall_gazebo.py %(gazeboworldMap)s %(connection)s
            ''' % locals()]) # rosrun p3dx_mover mover.py
                                #echo roslaunch p3dx_gazebo_mod gazebo_mod.launch world_name:="%(gazeboworldMap)s" x:=%(startPtFilex)f y:=%(startPtFiley)f z:=%(startPtFilez)f
                                #roslaunch p3dx_gazebo_mod gazebo_mod.launch world_name:="%(gazeboworldMap)s" x:=%(startPtFilex)f y:=%(startPtFiley)f z:=%(startPtFilez)f
        else:
            tabtitle.extend(['map creation + {startpt, map data} param push'])
            command.extend([''' cd rss_maps
                                python csvmapFromPolytopes.py %(mapFrom)s %(startPtFile)s %(csvparamsFile)s %(mobilesimMap)s %(csvOutFile)s %(boundaryedgesFile)s %(connection)s
            ''' % locals()])

        if (DLsolverUse == 'OMPL'): # solverToMapTypedict = {'OMPL': True, 'other': False}
            tabtitle.extend(['DL solver: OMPL actionlib node start'])
            command.extend([''' source %(topdir)s/devel/setup.bash
                                cd algorithms/rss_ompl_wrapper/scripts
                                rosrun rss_ompl_wrapper rss_ompl.py
            ''' % locals()])
        else:
            print("Unknown solver requested at DL...")
        
        if (HLsolverUse == 'OMPL'): # solverToMapTypedict = {'OMPL': True, 'other': False}
            tabtitle.extend(['HL solver: OMPL actionlib node start'])
            command.extend([''' source %(topdir)s/devel/setup.bash
                                cd algorithms/rss_ompl_wrapper/scripts
                                rosrun rss_ompl_wrapper rss_ompl.py
            ''' % locals()])
        else:
            print("Unknown solver requested at HL...")

        # for the above to all be tabs in the same window:
        terminal = ['gnome-terminal']
        for i in range(len(command)):
            terminal.extend(['--tab', '-e', '''
                bash -c '
                    %s
                    read
                '
            ''' % (command[i],), '-t', '%s' % (tabtitle[i],)])
            # 'read' to prevent each tab from closing immediately upon finish or error

        # note: under Fedora 20, have to push all to same window, multi-tabs, because subprocess.call()
        # will wait (under Fedora 20, not Ubuntu 14.04) until the first subprocess.call() window is closed
        # note: the same problem is seen with remote connections to Ubuntu 14.04

        # this will pop up the estimator, RL, HL, and DL as new windows from the last tab
        terminal.extend(['--tab', '-e', '''
            bash -c '
                ./subcall_estRLHLDL.py %(echo_input)d "%(Estcall)s" "%(RLcall)s" "%(HLcall)s" "%(DLcall)s" %(connection)s
                read
            '
        ''' % locals(), '-t', '%s' % ('popout gnome-terminal',)])

        subprocess.call(terminal)
        
        print("All init-scripts have now been called.")
        
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        try:
            print("Exception occurred, stopping script.")
        except NameError:
            print("NameError")

        sleep(3)
        #sys.exit(0)

# --EOF--

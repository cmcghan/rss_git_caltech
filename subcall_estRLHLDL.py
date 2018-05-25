#!/usr/bin/env python
# Copyright 2017-2018 by University of Cincinnati
# Copyright 2015-2016 by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite
"""
=== Summary of contents ===
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
# (note: this runs the psuedo-DL (not Matlab) and pseudo-estimator (not "real estimator"))

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

if __name__ == '__main__':
    
    print("Remember to run this from the top level of your rss_git directory, e.g.:")
    print("$ cd ~/catkin_ws/src/rss_work/rss_git")
    print("$ ./subcall_estRLHLDL.py echo_input startpt_offset RLcall HLcall DLcall connection")
    print("(defaults: echo_input=1, startpt_offset=13, RLcall='ReflexiveLayer_v3.0.py', HLcall='HabitualLayer_v3.0.py', DLcall='pseudoDL_v3.0.py', connection='ws://localhost:9090/')")
    print(" ")

    print("Starting subcall_estRLHLDL.py...")
    try:
        print("sys.argv = " + str(sys.argv))

        # python subcall_est_RLHLDL.py echo_input Estcall RLcall HLcall DLcall connection ...
        argvdefaults = [1,
                        "pseudoEstimator_v3.0.py",
                        "ReflexiveLayer_v3.0.py",
                        "HabitualLayer_v3.0.py",
                        "pseudoDL_v3.0.py",
                        "ws://localhost:9090/"] # default connection valid because starting rosbridge from this file :)
        [echo_input,
         Estcall,
         RLcall,
         HLcall,
         DLcall,
         connection] = fF.readArgvpieces(sys.argv,argvdefaults)
        echo_input = int(echo_input) # in case overwrote int with string
        #startpt_offset = int(startpt_offset) # in case overwrote int with string
        
        #
        # reset terminal string for new/next terminal run(s):
        #
        tabtitle = []
        command = []
        tabgeometry = []

        tabtitle.extend(['State Estimator'])
        command.extend([''' cd estimator
                            python %(Estcall)s %(echo_input)d %(connection)s
        ''' % locals()])
        tabgeometry.extend(['80x24+0+500']) # COLUMNSxROWS+X+Y
        
        tabtitle.extend(['Reflexive Layer'])
        command.extend([''' cd reflexive
                            python %(RLcall)s %(echo_input)d %(connection)s
        ''' % locals()])
        tabgeometry.extend(['80x24+800+500']) # COLUMNSxROWS+X+Y

        askforuserinput = -1
        tabtitle.extend(['Habitual Layer'])
        command.extend([''' cd habitual
                            python %(HLcall)s %(askforuserinput)d %(connection)s
        ''' % locals()])
        tabgeometry.extend(['80x24+800+0']) # COLUMNSxROWS+X+Y
        
        tabtitle.extend(['Deliberative Layer'])
        command.extend([''' cd deliberative
                            python %(DLcall)s %(askforuserinput)d %(connection)s
        ''' % locals()])
        tabgeometry.extend(['80x24+0+0']) # COLUMNSxROWS+X+Y

        # for the above to all be individual windows (-NOT- tabs in the same window):
        for i in range(len(command)):
            terminal = ['gnome-terminal', '--tab', '-e', '''
                bash -c '
                    printf """%s"""
                    %s
                    read
                '
            ''' % (command[i],command[i]), '-t', '%s' % (tabtitle[i],), '--geometry', '%s' % (tabgeometry[i],)]
            subprocess.call(terminal)
        # 'read' to prevent each tab from closing immediately upon finish or error
        
        print("All layer-scripts have now been called.")
        
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        try:
            print("Exception occurred, stopping script.")
        except NameError:
            print("NameError")

        sleep(3)
        #sys.exit(0)

# --EOF--

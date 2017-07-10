# rss_git_lite
Coming soon!

This repository will hold cleaned-up code from the version 2(.5) codeset from the [Resilient Space Systems](http://kiss.caltech.edu/techdev/systems/systems.html) project (Caltech + NASA JPL + MIT + WHOI)

This is the python-native (not ROS-native) version of the code created and maintained by the Caltech side of the effort.

The majority of this is "structural code" that handles rosbridge communications, layer module setup, function calls inside the layers, map setup, Gazebo-MobileSim setup, and similar.

The conversion of the code to a stripped-down and shareable version of itself is currently a work-in-progress, see below.

Done:
* rosbridge interfaces in python (non-rospy) (uploaded 2017-07-07)

Yet to be completed:
* ./multiopen_*.py and subcall scripts
* ./algorithms/rss_ompl_wrapper/*
* ./common/*.py (the rest of the directory)
* ./contrib/p3dx_gazebo_mod/* (most extracted as a separate repository already)
* ./deliberative/*.py (but -not- the subdirectories with the closed-source JPL and MIT algorithms -- those require permission for use)
* ./estimator/*.py
* ./habitual/*.py
* ./initialize-params/*.py and *.yaml
* ./input_files/*.txt (will be completely replaced)
* ./reflexive/*.py
* ./rss_maps/*.py and *.world and *.map and *.txt
* ./rss_msgs/* (will be completely replaced)
* ./rss_waypoints/*.txt (will be completely replaced)
* ./trackem and ./contrib/trackem_ros (will be forked from source and updated with new .py code)

--eof--

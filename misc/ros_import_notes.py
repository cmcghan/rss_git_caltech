# Copyright 2017 by University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite
"""
example(s) of straight python access to ROS python classes
(non-catkin, non-rospy to start with...)
"""

#import geometry_msgs
#help(geometry_msgs) # gives module msg
#import geometry_msgs.msg
#help(geometry_msgs.msg) # gives several module names
import geometry_msgs.msg._Point
#help(geometry_msgs.msg._Point) # Point() class exists inside this

# ROS msg python classes do not seem to have .__dict__ defined
# ...but should have: __getstate__, __setstate__, __str__

# this works:
testpt = geometry_msgs.msg._Point.Point()
print(testpt)
# sets x=0.0, y=0.0, z=0.0 internally to start with
testpt.x = 10.0
print(testpt)
# now is x=10.0, y=0.0, z=0.0

# alt.: from geometry_msgs.msg import Path
# lets you use Path() directly (no going through the _Path module)

# has .serialize() and .deserialize() classes that require inputs...
# unsure of the format / variable type of these currently, though
# -- bytestring and StringIO buffer for these, repsectively, apparently?
#    from StringIO import StringIO

#testpt.__getstate__ # bound method
testpt.__getstate__() # list of points, [x,y,z]

testpt.__setstate__([10.0,3.0,2.0]) # change all points at once via __getstate__ format
print(testpt)

testpt.__str__() # gives back a string of what it normally prints to screen

import json
json_str = '{"' + testpt.__str__().replace('\n',', "').replace(':','":') + '}' # converts to dictstr (json format, non-unicode)
#json_dict = json.loads(json_str) # converts string from ROS class to dict
json_dict = json.loads('{"' + testpt.__str__().replace('\n',', "').replace(':','":') + '}') # converts string from ROS class directly to dict
json_str2 = str(json_dict) # should be equivalent to / same as json_str EXCEPT values are unicode: u'stringdata'

# forget the stuff farther down, just use the rospy_message_converter to get the data back and forth:
# https://github.com/baalexander/rospy_message_converter
# $ sudo apt install ros-kinetic-rospy-message-converter
from rospy_message_converter import message_converter
holdit = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Point', json_dict)
type(holdit)
print(holdit)
holdit.x
# goes reverse direction, too: .convert_ros_message_to_dictionary()
from rospy_message_converter import json_message_converter
holdit2 = json_message_converter.convert_json_to_ros_message('geometry_msgs/Point', json_str)
type(holdit2)
print(holdit2)
holdit2.x
holdit3 = json_message_converter.convert_json_to_ros_message('geometry_msgs/Point', '{"x": 10.2, "y": 12.5, "z": 13.2}')
type(holdit3)
print(holdit3)
holdit3.x
# goes reverse direction, too: .convert_ros_message_to_json()

# note that the datatype -doesn't- need to have been loaded into the
# python session in order to be able to convert it and get the msg out
# nor does "source devel/setup.bash" need to be run at the commandline prior to starting python
# (...you just need "source /opt/ros/kinetic/setup.bash" ...or do you?)
holdit4 = json_message_converter.convert_json_to_ros_message('std_msgs/String', '{"data": "blahblehblah"}')
type(holdit4)
print(holdit4)
holdit4.data


#
# ignore stuff below this point
#

# incorrect
#json_str3 = '{"' + testpt.__str__().replace('\n',', "').replace(':','":') + '}' # converts to dictstr (json format, non-unicode)
#json_bytestr = json_str3.encode()
#testpt2 = geometry_msgs.msg._Point.Point()
#print(testpt2)
#testpt2.deserialize(json_bytestr) # unpack into this message


# will only work if .__dict__ works on the class, and it doesn't here:
#new_pt = geometry_msgs.msg._Point.Point()
#print(new_pt)
#for key,value in zip(json_dict.keys(),json_dict.values()):
#    set_attr(new_pt,key,value)
#    print(new_pt)

# instead, -IF AND ONLY IF- the ROS class takes arguments in alpha order (IT DOESN'T...)
#new_pt = geometry_msgs.msg._Point.Point()
#print(new_pt)
#valuelist = [json_dict[key] for key in sorted(json_dict.keys())] # for single-level dictionary
#new_pt.__setstate__(valuelist)
#print(new_pt)

# note that .__setstate__() will take what .__getstate__() gives it also
#import geometry_msgs.msg._TwistStamped
#testTw = geometry_msgs.msg._TwistStamped.TwistStamped()
#
##testTw.__setstate__(testTw.__getstate__())
#hold = testTw.__getstate__()
#print(hold)
##[seq: 0
##stamp: 
##  secs: 0
##  nsecs:         0
##frame_id: '', linear: 
##  x: 0.0
##  y: 0.0
##  z: 0.0
##angular: 
##  x: 0.0
##  y: 0.0
##  z: 0.0]
#type(hold) # is a list
#print(hold[0])
##seq: 0
##stamp: 
##  secs: 0
##  nsecs:         0
##frame_id: ''
#type(hold[0]) # is a std_msgs.msg._Header.Header
#
# ... so just passing a list to most ROS classes won't work at all
# it'll just overwrite the subtypes (e.g., Header)

# --eof--

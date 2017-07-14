#! /usr/bin/env python
# Copyright 2017 by University of Cincinnati
# Copyright 2015-2016 by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite
"""
=== Summary of contents ===
...

NOTE: as of 2017-07-06, the native rospy implementation section of this
wrapper class has NOT yet been implemented! (only stubs currently exist)
"""

#
# example Publisher (towards rospy implementation) below, modified from:
# http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
#
#import sys
#import std_msgs
#def testingpublisher():
#    rospy_pub_out = rospy.Publisher('/robot0/testtopic', std_msgs.String, queue_size=1)
#    rospy.init_node('testingpublisher', anonymous=True)
#    rate_of_send = rospy.Rate(15) # sending rate (sleeper function) = 15 Hz
#    while not rospy.is_shutdown():
#        send_str = "the time is now: " + str(rospy.get_time())
#        rospy.loginfo(send_str)
#        rospy_pub_out.publish(send_str)
#        rate_of_send.sleep()
#
#if __name__ == '__main__':
#    try:
#        testingpublisher()
#    except rospy.ROSInterruptException:
#        pass
#    sys.exit(0)


from time import sleep

class RosMsg(object):

    def __init__(self, connectlib, connection, pubsub, topicname, \
                 datatypename = None, un_pack_function = None):
        """
        calls "self.openNow()" then returns
        """
        self.openNow(connectlib, connection, pubsub, topicname, datatypename, un_pack_function)
        return
    
    def openNow(self, connectlib, connection, pubsub, topicname, \
                datatypename = None, un_pack_function = None):
        """
        input: connectlib is a string, one of: {'ws4py','rospy'}
               connection is a string (e.g., 'ws://localhost:9090/')
               pubsub is a string, one of: {'pub','sub'}
               topicname is a string
               datatype is a string of the form: 'xxx_msgs/DataTypeName'
                   (if default = None, topicname needs a default assigned)
               un_pack_function is a function (default = None)
                   (if 'pub', then sets up as publisher,
                    else if 'sub' then set up as a subscriber
                    
               note that a None input to un_pack_function sends it to
               look for an appropriate un/packing function:
               -- if datatypeName set: getws4pyDefaultUnPackFunctionForDatatype()
               -- if datatypeName not set, checks by topicname instead:
                      getws4pyDefaultUnPackFunctionForTopicname()
        
        algorithm: open/close connection to roscore via rospy (connectlib='rospy')
                   or rosbridge via Websockets-For-Python (connectlib='ws4py')
                   ==NOTE==: connectlib='rospy' is currently unsupported...
        
        output: 0 if successful, -1 if not
        """
        # check inputs; stop if one or more of essentials are None
        if (connectlib is None) or (connection is None) or (pubsub is None) or (topicname is None):
            print("Error: one of the required input arguments is None. Connection -not- initialized.")
            return -1
        # otherwise, continue...
        self.connectlib = connectlib
        self.connection = connection
        self.pubsub = pubsub
        self.topicname = topicname
        self.datatype = datatypename

        # must have some non-None topicname at this point...
        if (self.connectlib == 'ws4py'):
            from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS

            self.un_pack_function = un_pack_function
            if (self.un_pack_function is None): # if not given (basic datatype) then try to match type
                if (self.datatype is not None):
                    self.un_pack_function = getws4pyDefaultUnPackFunctionForDatatype(self.datatype,self.pubsub)
                    if (self.un_pack_function is None): # if no given/found function for datatype, stop
                        print('Error: unknown datatype %s' % self.datatype)
                        return -1
                else: #if (self.topicname is not None):
                    [self.un_pack_function,self.datatype] = getws4pyDefaultUnPackFunctionForTopicname(self.topicname,self.pubsub)
                    if (self.un_pack_function is None): # if no given/found function for topicname, stop
                        print('Error: no default datatype for topicname %s' % self.topicname)
                        return -1
            
            if (self.pubsub == 'sub'):
                self.rosconn = ws4pyROS.SubClient(self.connection)
            else: #if (self.pubsub == 'pub'):
                self.rosconn = ws4pyROS.PubClient(self.connection)
            self.rosconn.connect(self.topicname,self.datatype,self.un_pack_function)
            sleep(0.20)
            
        elif (self.connectlib == 'rospy'):
            import rospy
            import std_msgs
            import geometry_msgs
            import nav_msgs
            #import ...
            pass
            
        return 0
            
    def receive(self):
        """
        super-call, sits above self.copy_and_clear_received()
        -- this is the "safe" way to call it, checks the pub-sub status
        """
        data = None
        if (self.connectlib == 'ws4py'):
            if (self.pubsub == 'sub'):
                data = self.copy_and_clear_received()
            else:
                print("Error: attempted to subscribe to a publisher (%s)." % self.topicname)
        elif (self.connectlib == 'rospy'):
            # ???
            pass
        return data
    
    def copy_and_clear_received(self, olddata=None):
        """
        calls ws4pyROS routine of the same name
        
        returns data in the format specified by the unpacking function
        (self.un_pack_function) that was EITHER the default from
            getws4pyDefaultUnPackFunctionForDatatype() or
            getws4pyDefaultUnPackFunctionForTopicname()
        OR was user-set at initialization ("un_pack_function")
        
        if olddata is None, then returns None if no new data available
        if olddata is specified (not None), then if no data is
            available, olddata will be returned
        
        note: if self.connectlib='ws4py', then can call directly as:
              variablename.rosconn.copy_and_clear_received()
        (same input format as Pub/SubClient.copy_and_clear_received())
        """
        data = None
        if (self.connectlib == 'ws4py'):
            #import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
            data = self.rosconn.copy_and_clear_received(olddata)
            
        elif (self.connectlib == 'rospy'):
            #import rospy
            #import std_msgs
            #import geometry_msgs
            ##import ...
            # ???
            pass
        return data

    def send(self,datalist):
        """
        super-call, sits above self.send_pieces()
        -- this is the "safe" way to call it, checks the pub-sub status
        """
        if (self.connectlib == 'ws4py'):
            if (self.pubsub == 'pub'):
                self.send_pieces(datalist)
            else:
                print("Error: attempted to publish to a subscriber (%s)." % self.topicname)
        elif (self.connectlib == 'rospy'):
            # ???
            pass

    def send_pieces(self,datalist):
        """
        calls ws4pyROS routine of the same name
        
        sends datalist in the format specified by the packing function
        (self.un_pack_function) that was EITHER the default from
            getws4pyDefaultUnPackFunctionForDatatype() or
            getws4pyDefaultUnPackFunctionForTopicname()
        OR was user-set at initialization ("un_pack_function")
        
        note: if self.connectlib='ws4py', then can call directly as:
              variablename.rosconn.send_pieces(datalist)
        (same input format as Pub/SubClient.send_pieces())
        """
        if (self.connectlib == 'ws4py'):
            #import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
            self.rosconn.send_pieces(datalist)
            
        elif (self.connectlib == 'rospy'):
            import rospy
            import std_msgs
            import geometry_msgs
            #import ...
    
    def closeNow(self):
        """
        calls ws4pyROS routine closing() (self.rosconn.closing())
        
        closes the connection, and sets rosconn = None
        """
        if (self.connectlib == 'ws4py'):
            #import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
            self.rosconn.closing() # calls .close(), which calls .closed()
            #sleep(1) # lags things out greatly to have this in there, just write in sleep at the end post-call
            self.rosconn = None
            
        elif (self.connectlib == 'rospy'):
            import rospy
            import std_msgs
            import geometry_msgs
            #import ...
            # ???
            
    def isConnectionOpen():
        """
        checks if the connection is open (self.rosconn != None)
        """
        if (self.connectlib == 'ws4py'):
            if (self.rosconn is None):
                return False
            else:
                return True
        elif (self.connectlib == 'rospy'):
            # ???
            pass

def getws4pyDefaultUnPackFunctionForDatatype(datatype,pubsub):
    """
    input: datatype is a string of the form: 'xxx_msgs/DataTypeName'
           pubsub is a string, one of: {'pub','sub'}
           
    output: un_pack_function (python function) (None if no match)
    """
    from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
    
    pubdatatype = {#'std_msgs/Int16': ws4pyROS.pack_???,
                   'std_msgs/Int8': ws4pyROS.pack_runtype,
                   'std_msgs/Float32': ws4pyROS.pack_bsoc,
                   'std_msgs/Float64': ws4pyROS.pack_bvolt,
                   'std_msgs/Bool': ws4pyROS.pack_motorsstate,
                   'nav_msgs/Path': ws4pyROS.pack_nav_msgs_Path, #??? ***
                   #'std_msgs/String': ws4pyROS.pack_???
                   }

    subdatatype = {#'std_msgs/Int16': ws4pyROS.unpack_???,
                   'std_msgs/Int8': ws4pyROS.unpack_runtype,
                   'std_msgs/Float32': ws4pyROS.unpack_bsoc,
                   'std_msgs/Float64': ws4pyROS.unpack_bvolt,
                   'std_msgs/Bool': ws4pyROS.unpack_motorsstate,
                   'nav_msgs/Path': ws4pyROS.unpack_nav_msgs_Path, #??? ***
                   #'std_msgs/String': ws4pyROS.unpack_???
                   }
    
    un_pack_function = None
    if (pubsub == 'pub'):
        if (datatype in pubdatatype):
            un_pack_function = pubdatatype[datatype]
    else: #if (pubsub == 'sub'):
        if (datatype in subdatatype):
            un_pack_function = subdatatype[datatype]
        
    return un_pack_function

def getws4pyDefaultUnPackFunctionForTopicname(topicname,pubsub):
    """
    input: topicname is a string
           pubsub is a string, one of: {'pub','sub'}
           
    output: [un_pack_function (python function) (None if no match),
             datatype is a string of the form: 'xxx_msgs/DataTypeName']
    
    note: these default really should be read / loaded from a file
          rather than hardcoded here...
    """
    from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS

    pubtopicname = {'/robot0/waypoint_list': [ws4pyROS.pack_waypoints, 'nav_msgs/Path'],
                    '/robot0/waypointOne': [ws4pyROS.pack_onewaypoint, 'nav_msgs/Path'],
                    '/RosAria/cmd_vel': [ws4pyROS.pack_cmdvel, 'geometry_msgs/Twist'],
                    '/RosAria/pose': [ws4pyROS.pack_pose, 'nav_msgs/Odometry'],
                    '/trackem/trackempose': [ws4pyROS.pack_trackempose, 'nav_msgs/Odometry'],
                    #'/trackem/calpoints': [None,None], # not-existant: [ws4pyROS.pack_mtcalpoints, trackem_ros/MTCalPoints]
                    '/RosAria/bumper_state': [ws4pyROS.pack_bumperstate, 'rosaria/BumperState'],
                    '/RosAria/sonar': [ws4pyROS.pack_sonar, 'sensor_msgs/PointCloud'],
                    '/RosAria/battery_state_of_charge': [ws4pyROS.pack_bsoc, 'std_msgs/Float32'],
                    '/RosAria/battery_voltage': [ws4pyROS.pack_bvolt, 'std_msgs/Float64'],
                    '/RosAria/battery_recharge_state': [ws4pyROS.pack_brecharge, 'std_msgs/Int8'],
                    '/RosAria/motors_state': [ws4pyROS.pack_motorsstate, 'std_msgs/Bool']
                    }
    
    subtopicname = {'/robot0/waypoint_list': [ws4pyROS.unpack_waypoints, 'nav_msgs/Path'],
                    '/robot0/waypointOne': [ws4pyROS.unpack_onewaypoint, 'nav_msgs/Path'],
                    '/RosAria/cmd_vel': [ws4pyROS.unpack_cmdvel, 'geometry_msgs/Twist'],
                    '/RosAria/pose': [ws4pyROS.unpack_pose, 'nav_msgs/Odometry'],
                    '/trackem/trackempose': [ws4pyROS.unpack_trackempose, 'nav_msgs/Odometry'],
                    '/trackem/calpoints': [ws4pyROS.unpack_mtcalpoints, 'trackem_ros/MTCalPoints'],
                    '/RosAria/bumper_state': [ws4pyROS.unpack_bumperstate, 'rosaria/BumperState'],
                    '/RosAria/sonar': [ws4pyROS.unpack_sonar, 'sensor_msgs/PointCloud'],
                    '/RosAria/battery_state_of_charge': [ws4pyROS.unpack_bsoc, 'std_msgs/Float32'],
                    '/RosAria/battery_voltage': [ws4pyROS.unpack_bvolt, 'std_msgs/Float64'],
                    '/RosAria/battery_recharge_state': [ws4pyROS.unpack_brecharge, 'std_msgs/Int8'],
                    '/RosAria/motors_state': [ws4pyROS.unpack_motorstate, 'std_msgs/Bool']
                    }
    
    un_pack_function = None; datatype = None
    if (pubsub == 'pub'):
        if (topicname in pubtopicname):
            [un_pack_function, datatype] = pubtopicname[topicname]
    else: #if (pubsub == 'sub'):
        if (topicname in subtopicname):
            [un_pack_function, datatype] = subtopicname[topicname]

    return [un_pack_function, datatype]

def sendRunTypeValueAndShutdownWs4py(runtypeSend,connection,ws_runtype_in,ws_runtype_out=None):
    """
    input: runtypeSend is an int (the value to send)
           connection is a string (e.g., 'ws://localhost:9090/')
           ws_runtype_in is a RunType RosMsg() sub
           ws_runtype_out is a RunType RosMsg() pub (not required)
           
    algorithm: sends runtypeSend integer over '/robot0/runtype'
               waits until it's been received again (/ knows was sent)
               then shutsdown both channels
    
    output: n/a (closes ws_runtype_in and ws_runtype_out when done)
    """
    from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
    
    if (ws_runtype_out == None):
        ws_runtype_out = RosMsg('ws4py', connection, 'pub', '/robot0/runtype', 'std_msgs/Int8', ws4pyROS.pack_runtype)
        
    if (ws_runtype_in == None):
        ws_runtype_in = RosMsg('ws4py', connection, 'sub', '/robot0/runtype', 'std_msgs/Int8', ws4pyROS.unpack_runtype)
        
    ws_runtype_out.send_pieces(runtypeSend)
    while(1):
        runtype = ws_runtype_in.copy_and_clear_received()
        if (runtype == runtypeSend):
            sleep(2)
            break;
        else:
            print("Waiting for runtype=%d stop command to propagate through rosbridge_server..." % runtypeSend)
            sleep(0.1)

    ws_runtype_in.closeNow()
    ws_runtype_out.closeNow()
    sleep(1)

class BlankClass(object): # blank class object allows .___ to be added to this object/class...
    pass

#
# an example of ws4py socket comms use, talking to a rosbridge_server on port 9090:
#
if __name__=="__main__":

    # Note that the following should work so long as you modify the sys.path
    # to include the directory which holds the top-level directory of each
    # package that you need, and you'll need to include __init__.py in each
    # directory inside that "package" so that you can import what you need.
    #
    # This should be done for every python-executable file. The executable /
    # file that invokes the python interpreter needs to be able to find all
    # packages that the entire run needs. Changing sys.path from this invoked
    # file (relative to the invoked file) works because everything in a
    # python (interpreter run) instance sees the same sys.path.
    #
    import sys # for sys.exit() and sys.path.append()
    file_dir = sys.path[0] # *** initialized on program startup, directory of the script used to invoke the python interpreter ***
    sys.path.append(file_dir + '/../..') # modify sys.path to include directory containing rss_git_lite "package"
    #print("sys.path = %r\n" % sys.path)

    from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
#    from rss_git_lite.common import rosConnectWrapper as rC # gives RosMsg, call via: rC.RosMsg

    # works:
    #from nav_msgs.msg import Odometry
    #loc = Odometry()
    #loc.pose.pose.position.x = 10.05
    #loc.pose.pose.position.y = -5.63
    #print('%r' % loc)
    #print('%s' % loc)

#    from geometry_msgs.msg import Twist
#    fullpose = Twist()
#    fullpose.linear.x = 0.1
    fullpose = BlankClass()
    fullpose.linear = BlankClass()
    fullpose.angular = BlankClass()
    fullpose.linear.x = 0.1
    fullpose.linear.y = 0
    fullpose.linear.z = 0
    fullpose.angular.x = 0
    fullpose.angular.y = 0
    fullpose.angular.z = 0

    from rss_git_lite.common import getConnectionIPaddress as gC
    connection = gC.getConnectionIPaddress()
    #connection = "ws://localhost:9090/" # example string for connecting to rosbridge_server
    #connection = gC.getConnectionIPaddress(3) # this should give back: "ws://localhost:9090/"

    try:
        ws = RosMsg('ws4py', connection, 'sub', '/robot0/runtype', 'std_msgs/Int8', ws4pyROS.unpack_runtype)
        # retrieves data if it exists, None if no receipt since last call
        #runtype = ws.copy_and_clear_received() # receive "directly", no checks on channel pub/sub status
        runtype = ws.receive() # "safer"-receive
        #print("runtype = %r " % runtype)

        #
        # if you are running MobileSim and RosAria, you could use the below command to control the robot's motion:
        #
        ws2 = RosMsg('ws4py', connection, 'pub', '/RosAria/cmd_vel', 'geometry_msgs/Twist', ws4pyROS.pack_cmdvel)
        #
        # if you are running p3dx in gazebo, you could use this ws2 instead set up control the robot's motion:
        #
        #ws2 = RosMsg('ws4py', connection, 'pub', '/cmd_vel', 'geometry_msgs/Twist', ws4pyROS.pack_cmdvel)
        # then set up and send the command:
        cmdvelcmd = {'linear': {'x': fullpose.linear.x, 'y': fullpose.linear.y, 'z': fullpose.linear.z}, \
                     'angular': {'x': fullpose.angular.x, 'y': fullpose.angular.y, 'z': fullpose.angular.z}}
        ws2.rosconn.send_message(cmdvelcmd) # to send directly in JSON format through ws4py connection interface
        #print("cmdvelcmd sent!\n%r" % cmdvelcmd)
        
        #
        # example code for sending waypoints using packing function to translate datalist to JSON format
        #
        ws_waypts_out = RosMsg('ws4py', connection, 'pub', '/robot0/waypoint_list', 'nav_msgs/Path', ws4pyROS.pack_waypoints)
        waypoints_to_send = [ [0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0], [0.0, 0.0] ] # list of lists
        # send waypoint(s), waytype=1 ([x,y] points, no set orientation)
        #ws_waypts_out.send_pieces([waypoints_to_send, 1]) # send directly, no checks on channel pub/sub status
        ws_waypts_out.send([waypoints_to_send, 1]) # "safer"-send
        #print("waypoints_to_send sent!\n%r" % waypoints_to_send)
        
        sleep(3)
    except KeyboardInterrupt:
        ws.closeNow()
        ws2.closeNow()
        ws_waypts_out.closeNow()

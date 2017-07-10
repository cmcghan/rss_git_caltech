# Copyright 2017 by University of Cincinnati
# Copyright 2014-2016 by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite
"""
=== Summary of contents ===
"""

from json import dumps
from json import loads
from ws4py.client.threadedclient import WebSocketClient
from time import sleep
from time import time
import math
import sys # for sys.exit()

class GetLoggersClient(WebSocketClient):
  
    def get_loggers(self):
        msg = {'op': 'call_service', 'service': '/rosout/get_loggers'}
        self.send(dumps(msg))
  
    def opened(self):
        print "Connection opened..."
        self.get_loggers()
  
    def closed(self, code, reason=None):
        print code, reason
  
    def received_message(self, m):
        print "Received:", m

class ConvertJSON(object):
    def __init__(self,jsonObjThing):
        self.__dict__ = loads(str(jsonObjThing))

class CopyJSON(object):
    def __init__(self,jsonObj):
        self.__dict__ = jsonObj




class SubClient(WebSocketClient):
    """
    template for this stuff is:

    # Recv??Client
    # 'topic': '', 'type': ''

    def unpack_??(dictmsg):
        ?? = dictmsg['msg']['??']
        return ??

    def pack_??(datalist):
        ?? = datalist[0]
        rosmsg = {'???': ??}
        return rosmsg
    """
    
    def __init__(self, arg):
        super(SubClient,self).__init__(arg) # super inheritance -- sends the connection address (e.g. 'ws://localhost:9090/')
        self.dictmsg = None # adding element(s) to base class to hold the data received # raw json msg data in
        #self.rosmsg = None # adding element(s) to base class to hold the data received # raw ros msg data in
        from multiprocessing import Lock # .acquire(block={true,false},timeout={None,int})
        self.jsonMutex = Lock()
        self.topicstr = None
        self.typestr = None
        self.unpackingfunction = None
        self.firstrun = None # necessary for some conversions that require time or other things to be kept track of later..., e.g. unpack_pose2d_as_pose()

    def connect(self, topicstr, typestr, unpackingfunction): # call this after __init__ before anything else
        self.topicstr = topicstr
        self.typestr = typestr
        self.unpackingfunction = unpackingfunction
        super(SubClient,self).connect() # super inheritance

    def opened(self): # occurs automatically after .connect() if .connect() is successful
        self.dictmsg = None
        #self.rosmsg = None
        #print "DEBUG: Connection opened..."
        msg = {'op': 'subscribe', 'topic': self.topicstr, 'type': self.typestr, 'queue_length': 1} # supposed to be set to 1 by default (no queue), but rosbridge, so...
        self.send(dumps(msg))
  
    def closing(self): # calls .close(), which calls .closed()
        msg = {'op': 'unsubscribe', 'topic': self.topicstr}
        self.send(dumps(msg))
        sleep(0.2)
        self.close()

    def closed(self, code, reason=None):
        #print code, reason
        #print "DEBUG: %r %r" % (code, reason)
        pass

    def received_message(self, m): # will be called automatically after a successful .connect() (and .opened())
        #print "Received:", m # this clears out the string apparently?
        #print("Received: %r" % m) # while this does not -- Received: <ws4py.messaging.TextMessage object at 0x8d7550>
        self.jsonMutex.acquire(True)
        if m.is_text:
#            msg_str = m.data.decode("utf-8")
#            print("Received: %s" % msg_str)
#            dictmsg = loads(msg_str)
            self.dictmsg = loads(str(m))
            #self.rosmsg = self.dictmsg['msg']
            self.jsonMutex.release()

    def copy_and_clear_received(self,olddata=None):
        """
        if olddata = None (or not given) then returns None if no new data
            (will have to check if output is None before doing an overwrite)
        if olddata != None then returns olddata (previous info) if no new data
            (will not overwrite data if do '[dataheld] = ws.copy_and_clear_received(dataheld])')
        """
        self.jsonMutex.acquire(True)
        if (self.dictmsg == None):
            self.jsonMutex.release()
            return olddata
        else:
            unpackeddata = self.unpackingfunction(self.dictmsg)
            self.dictmsg = None
            #self.rosmsg = None
            self.jsonMutex.release()
            return unpackeddata

class PubClient(WebSocketClient):
    """
    template for this stuff is:

    # Recv??Client
    # 'topic': '', 'type': ''

    def unpack_??(dictmsg):
        ?? = dictmsg['msg']['??']
        return ??

    def pack_??(datalist):
        ?? = datalist[0]
        rosmsg = {'???': ??}
        return rosmsg
    """
    
    def __init__(self, arg):
        super(PubClient,self).__init__(arg) # super inheritance -- sends the connection address (e.g. 'ws://localhost:9090/')
        self.topicstr = None
        self.typestr = None
        self.packingfunction = None
          
    def connect(self, topicstr, typestr, packingfunction): # call this after __init__ before anything else
        self.topicstr = topicstr
        self.typestr = typestr
        self.packingfunction = packingfunction
        super(PubClient,self).connect() # super inheritance

    def opened(self): # occurs automatically after .connect() if .connect() is successful
        #print "DEBUG: Connection opened..."
        msg = {'op': 'advertise', 'topic': self.topicstr, 'type': self.typestr}
        self.send(dumps(msg))

    def closing(self): # calls .close(), which calls .closed()
        msg = {'op': 'unadvertise', 'topic': self.topicstr}
        self.send(dumps(msg))
        sleep(0.2)
        self.close()

    def closed(self, code, reason=None):
        #print code, reason
        #print "DEBUG: %r %r" % (code, reason)
        pass
  
    def received_message(self, m): # ...should never be called?
        print "RunType response Received:", m

    def send_message(self,rosmsg): # rosmsg is json format by this point
#        msg = {'op': 'publish', 'topic': '/robot0/runtype', 'msg': {'data': data_value} }
        msg = {'op': 'publish', 'topic': self.topicstr, 'msg': rosmsg }
        self.send(dumps(msg))

    def send_pieces(self,datalist):
        rosmsg = self.packingfunction(datalist)
        self.send_message(rosmsg)

# 'type': 'std_msgs/Int8'
def unpack_std_msgs_Int8(dictmsg):
    data = dictmsg['msg']['data']
    return data

def pack_std_msgs_Int8(datalist):
    data = datalist
    rosmsg = {'data': data}
    return rosmsg

# RecvRuntypeClient
# 'topic': '/robot0/runtype', 'type': 'std_msgs/Int8'

def unpack_runtype(dictmsg):
    runtype = unpack_std_msgs_Int8(dictmsg)
    return runtype

def pack_runtype(datalist):
    rosmsg = pack_std_msgs_Int8(datalist)
    return rosmsg

# RecvWayPointsClient
# 'topic': '/robot0/waypoint_list', 'type': 'nav_msgs/Path'

def unpack_waypoints(dictmsg):
    current_waypoints = unpack_nav_msgs_Path_stripped(dictmsg['msg'])
    return current_waypoints

def json_header(t,frame_id=''):
    secs = int(t)
    nsecs = int((t - secs) * 1000000000)
    inc = int(t*10 + int(nsecs/100000000)) # mult. chunks both by 10
    json_header = {'seq': inc, 'stamp': {'secs': secs, 'nsecs': nsecs}, 'frame_id': frame_id}
    return json_header

def pack_waypoints(datalist):
    """
    datalist = [ [ [x,y,...], ...] , waytype ]

    waytype = 1 is [ [x,y], ...] format
    waytype = 2 is [ [x,y,z,qx,qy,qz,qw], ...] format
    
    if no waytype is given(??) then default waytpe=1
    """
    # different bit here, need to pack the datalist a bit harder:
    #waypoints_var = datalist[0]
    if (len(datalist)<2): # this isn't quite right, will get confused if only list-of-lists given
        waypoints_var = datalist
        waytype=1
    else:
        #waypoints_var = datalist[0]
        #waytype = datalist[1]
        [waypoints_var, waytype] = datalist[0:2] # = datalist

    poses_json_list = []
    for xyzqt in waypoints_var : # list of tuples or list of lists
        if (waytype == 1):
            #x = xyzqt[0] ; y = xyzqt[1]
            #z = 0 ; qx = 0 ; qy = 0 ; qz = 0 ; qw = 0 ; t = 0
            [x,y] = xyzqt[0:2]
            z = qx = qy = qz = qw = t = 0
        elif (waytype == 2):
            #x = xyzqt[0] ; y = xyzqt[1] ; z = xyzqt[2] ; qx = xyzqt[3] ; qy = xyzqt[4] ; qz = xyzqt[5] ; qw = xyzqt[6]
            #t = xyzqt[7]
            [x,y,z,qx,qy,qz,qw,t] = xyzqt[0:8] # = xyzqt
        waypoints_json_header = json_header(t)
        waypoints_json_pose = {'position': {'x': x, 'y': y, 'z': z}, 'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw}}
        poses_json_list.append({'header': waypoints_json_header, 'pose': waypoints_json_pose })
    t = 0 # ???
    waypoints_json_header = json_header(t)
    waypoints_json = {'header': waypoints_json_header, 'poses': poses_json_list}
    return waypoints_json

# RecvOneWayPointClient
# 'topic': '/robot0/waypointOne', 'type': 'nav_msgs/Path'

def unpack_onewaypoint(dictmsg):
    onewaypt_msg = dictmsg['msg']['poses']
    if len(onewaypt_msg)==1:
        current_onewaypoint = unpack_waypoints(dictmsg)
    else:
        print("Error, gave more than one waypoint to onewaypoint type send. Not sending.")
        return None
    return current_onewaypoint

def pack_onewaypoint(datalist):
    """
    waytype = 1 is [ [x,y], ...] format
    waytype = 2 is [ [x,y,z,qx,qy,qz,qw], ...] format
    """
    # different bit here, need to pack the datalist a bit harder:
    if (len(datalist)<2):
    #    onewaypoint_var = datalist[0]
        onewaypoint_var = datalist
        if len(onewaypoint_var)==1:
            onewaypoint_json = pack_waypoints(datalist)
        else:
            print("Error, gave more than one waypoint to onewaypoint type send. Not sending.")
            return None
    else:
        onewaypoint_var = datalist[0]
        if len(onewaypoint_var)==1:
            onewaypoint_json = pack_waypoints(datalist)
        else:
            print("Error, gave more than one waypoint to onewaypoint type send. Not sending.")
            return None
        
    return onewaypoint_json

# RecvCmdVelClient
# 'topic': '/RosAria/cmd_vel', 'type': 'geometry_msgs/Twist'

def unpack_cmdvel(dictmsg):
    cmdvel_msg = dictmsg['msg']
##    cmdvel = CopyJSON(cmdvel_json['msg'])
    cmdvel = [cmdvel_msg['linear']['x'] ,
                cmdvel_msg['linear']['y'] ,
                cmdvel_msg['linear']['z'] ,
                cmdvel_msg['angular']['x'] ,
                cmdvel_msg['angular']['y'] ,
                cmdvel_msg['angular']['z'] ]
#    print("cmdvel = %r" % cmdvel)
    #print("cmdvel_msg = %r" % cmdvel_msg)
        
    return cmdvel

def pack_cmdvel(datalist): # this is a little different than before, need to put in a list
    """
    datalist = [vx,vy,vz,wx,wy,wz]
    """
    vx,vy,vz,wx,wy,wz = datalist
    cmdvel_json = {'linear': {'x': vx, 'y': vy, 'z': vz}, 'angular': {'x': wx, 'y': wy, 'z': wz}}
    return cmdvel_json

def unpack_nav_msgs_Odometry_stripped(msg):
    """
    input: dictmsg['msg']
    output: pose = [seq,secs,nsecs,x,y,z,qx,qy,qz,qw]
    """
    pose = [msg['header']['seq'] ,
            msg['header']['stamp']['secs'] ,
            msg['header']['stamp']['nsecs'] ,
            msg['pose']['pose']['position']['x'] ,
            msg['pose']['pose']['position']['y'] ,
            msg['pose']['pose']['position']['z'] ,
            msg['pose']['pose']['orientation']['x'] ,
            msg['pose']['pose']['orientation']['y'] ,
            msg['pose']['pose']['orientation']['z'] ,#]
            msg['pose']['pose']['orientation']['w'] ] # now also included in output
    return pose

# RecvPoseClient
# 'topic': '/RosAria/pose', 'type': 'nav_msgs/Odometry'

def unpack_pose(dictmsg):
    """
    calls unpack_nav_msgs_Odometry_stripped()
    input: dictmsg
    output: pose = [seq,secs,nsecs,x,y,z,qx,qy,qz,qw]
    """
    #pose_msg = dictmsg['msg']
#    print "Pose response Received:", m
#    pose_msg = (ConvertJSON(m))['msg']
    pose = unpack_nav_msgs_Odometry_stripped(dictmsg['msg'])
    return pose # [seq,secs,nsecs,x,y,z,qx,qy,qz,qw]

def pack_pose(datalist): # this is a little different than before, need to put in a list
    """
    input: datalist = [x,y,z,qx,qy,qz,qw,t]
    output: pose_json (dictionary)
    note that covariance is zeroed out here (default value = list of 0's)
    """
    x,y,z,qx,qy,qz,qw,t = datalist
    pose_json_header = json_header(t)
    covar = range(0,36) # should give 0 to, not through, 36, a.k.a. 0 through 35 (-- for 36 values)
    pose_json = {'header': pose_json_header, 'child_frame_id': '', 
                'pose': {'pose': {'position': {'x': x, 'y': y, 'z': z},
                                'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw}},
                                'covariance': covar},
                'twist': {'twist' : {'linear': {'x': 0, 'y': 0, 'z': 0},
                                    'angular': {'x': 0, 'y': 0, 'z': 0}} ,
                        'covariance': covar}
                }
    return pose_json

def unpack_geometry_msgs_Twist(dictmsg):
    """
    input: dictmsg
    output: twist = [x,y,z,r,p,h] (velocities or acceleration generally)
    """
    twist_msg = dictmsg['msg']
    twist = [twist_msg['linear']['x'],
             twist_msg['linear']['y'],
             twist_msg['linear']['z'],
             twist_msg['angular']['x'],
             twist_msg['angular']['y'],
             twist_msg['angular']['z'] ]
    return twist

def pack_geometry_msgs_Twist(datalist):
    """
    input: datalist = [x,y,z,r,p,h] (velocities or acceleration generally)
    output: twist_json (dictionary)
    """
    [x,y,z,r,p,h] = datalist
    twist_json = {'linear': {'x': x, 'y': x, 'z': z},
                  'angular': {'x': r, 'y': p, 'z': h}}
    return twist_json

# RecvTrackemPoseClient
# 'topic': '/trackem/trackempose', 'type': 'nav_msgs/Odometry'

def unpack_trackempose(dictmsg):
    """calls unpack_pose()"""
    thepose = unpack_pose(dictmsg)
    return thepose

def pack_trackempose(datalist):
    """calls pack_pose()"""
    pose_json = pack_pose(datalist)
    return pose_json

# RecvMTCalPointsClient
# 'topic': '/trackem/calpoints', 'type': 'trackem_ros/MTCalPoints'

def unpack_mtcalpoints(dictmsg):
    """
    input: dictmsg
    output: current_points (list-of-lists: [[x,y],...]), mtcalpoints_head (list: [cam_id,framenumber,timestamp])
    """
    current_points = []
    mtcalpoints_msg = dictmsg['msg']
#    print "MTCalPoints response Received:", m
#    mtcalpoints_msg = (ConvertJSON(m))['msg']
    mtcalpoints_head = [mtcalpoints_msg['cam_id'] , mtcalpoints_msg['framenumber'] , mtcalpoints_msg['timestamp'] ]
    
    calpt_msglist = mtcalpoints_msg['points']
    i = 0
    for xlist_msg in calpt_msglist:
        waypt_xy = [xlist_msg['x'], xlist_msg['y']]
        current_points.append(tuple(waypt_xy))

    return current_points, mtcalpoints_head

#def pack_mtcalpoints(datalist):
#    ?? = datalist[0]
#    rosmsg = {'???': ??}
#    return rosmsg

# RecvBumperStateClient
# 'topic': '/RosAria/bumper_state', 'type': 'rosaria/BumperState'

class Stam(object): # note that .__dict__ is defined for this :)
    def __init__(self):
        self.secs = int()
        self.nsecs = int()

class Head(object): # note that .__dict__ is defined for this :)
    def __init__(self):
        self.seq = int()
        self.stamp = Stam()

class BumperState(object): # note that .__dict__ is defined for this :)
    def __init__(self):
        self.header = Head()
        self.front_bumpers = bool()
        self.rear_bumpers = bool()

def unpack_bumperstate(dictmsg):
    """
    input: dictmsg
    output: bumperstate (class BumperState; basic, not rospy/rosaria one)
    """
    bumperstate_msg = dictmsg['msg']
#    print "BumperState response Received:", m
#    bumperstate_msg = (ConvertJSON(m))['msg']
    bumperstate.header.seq = bumperstate_msg['header']['seq']
    bumperstate.header.stamp.secs = bumperstate_msg['header']['stamp']['secs']
    bumperstate.header.stamp.nsecs = bumperstate_msg['header']['stamp']['nsecs']
    bumperstate.front_bumpers = bumperstate_msg['front_bumpers']
    bumperstate.rear_bumpers = bumperstate_msg['rear_bumpers']
    #print("bumper_state = %r" % bumperstate.__dict__)
    return bumperstate # ???

def pack_bumperstate(datalist): # this is a little different than before, need to put in a list
    """
    input: datalist = [front_bumpers,rear_bumpers,t]
    output: bumperstate_json (dictionary)
    """
    front_bumpers,rear_bumpers,t = datalist
    bumperstate_json_header = json_header(t)
    bumperstate_json = {'header': bumperstate_json_header, 'front_bumpers': front_bumpers, 'rear_bumpers': rear_bumpers}
    return bumperstate_json

# RecvSonarClient
# 'topic': '/RosAria/sonar', 'type': 'sensor_msgs/PointCloud'

class Point32(object):
    def __init__(self):
        self.x = float()
        self.y = float()
        self.z = float()

class ChannelFloat32(object):
    def __init__(self,thename='',thevalues=[]):
        self.name = string(thename)
        self.values = float(thevalues)

class PointCloud(object):
    def __init__(self):
        self.header = Head()

def unpack_sonar(dictmsg):
    """
    input: dictmsg
    output: current_sonarpts (list of [x,y,z]'s), current_channels (list of [name,values]'s)
    """
    current_sonarpts = []
    current_channels = []
    #print("msg = %r" % dictmsg) # debug line
    sonar_msglist = dictmsg['msg']['points']
    for xlist_msg in sonar_msglist:
        sonarpt_xyz = [xlist_msg['x'], xlist_msg['y'], xlist_msg['z']]
        current_sonarpts.append(sonarpt_xyz)
    channel_msglist = dictmsg['msg']['channels']
    for xlist_msg in channel_msglist:
        current_channels.append(ChannelFloat32(xlist_msg['name'],xlist_msg['values']))

    return current_sonarpts, current_channels # ???

def pack_sonar(datalist): # this is a little different than before, need to put in a list
    """
    input: datalist = [t_var,points_var,name_var,values_var]
    output: sonar_json (dictionary)
    """
    t_var,points_var,name_var,values_var = datalist
    sonar_json_header = json_header(t_var)
    points_json_list = []
    for xyz in points_var : # list of lists
        sonar_json_points = {'x': xyz[0], 'y': xyz[1], 'z': xyz[2]}
        points_json_list.append(sonar_json_points)
    channels_json_list = []
    for i in range(len(name_var)): # list of strings, e.g. name_var = ['test1','test2',...]
        name = name_var[i] ; values = values_var[i]
        sonar_json_channels = {'name': name, 'values': values}
        channels_json_list.append(sonar_json_channels)
    sonar_json = {'header': sonar_json_header, 'points': points_json_list, 'channels': channels_json_list}
    return sonar_json

# RecvBSOCClient
# 'topic': '/RosAria/battery_state_of_charge', 'type': 'std_msgs/Float32'

def unpack_bsoc(dictmsg):
    """
    input: dictmsg
    output: bsoc (float32)
    """
    bsoc = dictmsg['msg']['data']
    #print("bsoc = %r" % bsoc)
#    print "Battery State Of Charge response Received:", m
#    bsoc_msg = (ConvertJSON(m))['msg']
    return bsoc # ???

def pack_bsoc(datalist):
    """
    input: datalist = percentBL (float32)
    output: bsoc_json (dictionary)
    """
    percentBL = datalist
    bsoc_json = {'data': percentBL}
    return bsoc_json

# RecvBVoltClient
# 'topic': '/RosAria/battery_voltage', 'type': 'std_msgs/Float64'

def unpack_bvolt(dictmsg):
    """
    input: dictmsg
    output: bvolt (float64)
    """
    bvolt = dictmsg['msg']['data']
    #print("bvolt = %r" % bvolt)
#    print "Battery Voltage response Received:", m
#    bvolt_msg = (ConvertJSON(m))['msg']
#    bvolt = bvolt_msg['data']
    return bvolt

def pack_bvolt(datalist):
    """
    input: datalist = battDCvoltage (float64)
    output: bvolt_json (dictionary)
    """
    battDCvoltage = datalist
    bvolt_json = {'data': battDCvoltage}
    return bvolt_json

# RecvBRechargeClient
# 'topic': '/RosAria/battery_recharge_state', 'type': 'std_msgs/Int8'

def unpack_brecharge(dictmsg):
    """
    input: dictmsg
    output: brecharge (int8)
    """
    brecharge = dictmsg['msg']['data']
    return brecharge

def pack_brecharge(datalist):
    """
    input: datalist = battery_recharge_state (int8)
    output: brecharge_json (dictionary)
    """
    battery_recharge_state = datalist
    brecharge_json = {'data': battery_recharge_state}
    return brecharge_json

# RecvMotorsStateClient
# 'topic': '/RosAria/motors_state', 'type': 'std_msgs/Bool'

def unpack_motorsstate(dictmsg):
    """
    input: dictmsg
    output: motorsstate (bool)
    """
    motorsstate = dictmsg['msg']['data']
    return motorsstate

def pack_motorsstate(datalist):
    """
    input: datalist = motors_state (bool)
    output: motorsstate_json (dictionary)
    """
    motors_state = datalist
    motorsstate_json = {'data': motors_state}
    return motorsstate_json

def sendRunTypeValueAndShutdown(runtypeSend,connection,ws_runtype_in,ws_runtype_out=None): # expects an int, a RunType PubClient() (...and a RunType SubClient())
    # connection = 'ws://192.168.1.101:9090/'  # example of string contents
    if (ws_runtype_out == None):
        ws_runtype_out = PubClient(connection)
        ws_runtype_out.connect('/robot0/runtype','std_msgs/Int8',pack_runtype)
        sleep(0.20) # may need slightly more time over the network the first time

    if (ws_runtype_in == None):
        ws_runtype_in = SubClient(connection)
        ws_runtype_in.connect('/robot0/runtype','std_msgs/Int8',unpack_runtype)
        sleep(0.20) # may need slightly more time over the network the first time

    ws_runtype_out.send_pieces(runtypeSend)
    while(1):
        runtype = ws_runtype_in.copy_and_clear_received()
        if (runtype == runtypeSend):
            sleep(2)
            break;
        else:
            print("Waiting for runtype=%d stop command to propagate through rosbridge_server..." % runtypeSend)
            sleep(0.1)

    ws_runtype_in.closing() # calls .close(), which calls .closed()
    ws_runtype_out.closing() # calls .close(), which calls .closed()
    sleep(1)

# 'topic': '/robot0/waypoint_list', 'type': 'nav_msgs/Path'
    
def unpack_nav_msgs_Path_stripped(msg):
    """
    input: dictmsg['msg']
    output: current_waypoints (list of tuples: [(x,y),(x,y),...])
    """
    # header_msg = msg['header']
    waypt_msglist = msg['poses']
    current_waypoints = []
    for xlist_msg in waypt_msglist:
#       seq = xlist_msg['header']['seq'] # may need to reorder in proper sequence...
        xyzqt_msg = xlist_msg['pose']['position']
        waypt_xy = [xyzqt_msg['x'], xyzqt_msg['y']]
        current_waypoints.append(tuple(waypt_xy))
#   current_waypoints = [tuple(xlist) for xlist in self.waypt_msglist]
    #print("current_waypoints = %r" % current_waypoints)
    return current_waypoints

def pack_nav_msgs_Path(datalist):
    """
    calls pack_waypoints(datalist)
    
    datalist = [ [ [x,y,...], ...] , waytype ]

    waytype = 1 is [ [x,y], ...] format
    waytype = 2 is [ [x,y,z,qx,qy,qz,qw], ...] format
    
    if no waytype is given(??) then default waytpe=1
    """
    waypoints_json = pack_waypoints(datalist)
    return waypoints_json

# ...
# '/robot0/pathlength','std_msgs/Float64'

def unpack_pathlength(dictmsg):
    """
    input: dictmsg
    output: pathlength (float64)
    """
    pathlength = dictmsg['msg']['data']
    return pathlength

def pack_pathlength(datalist):
    """
    input: datalist = pathlength (float64)
    outputL pathlength_json (dictionary)
    """
    pathlength = datalist
    pathlength_json = {'data': pathlength}
    return pathlength_json

# ...
# '/new/overhead_tracking','geometry_msgs/Pose2D'    
def unpack_pose2d_as_pose(dictmsg):
    """
    input: dictmsg (pose2d json dict format)
    output: pose = [seq,secs,nsecs,x,y,z,qx,qy,qz,qw]
    """
    if (self.firstrun is None):
        self.firstrun = 0 # use as seq
    else:
        self.firstrun += 1
    self.t = time() # grabs system time, closest thing we have to deal with...
    seq = self.firstrun
    x = dictmsg['msg']['x']
    y = dictmsg['msg']['y']
    h = dictmsg['msg']['theta']
    # now, transform to 3d coords
    z = 0
    #t = secs + nsecs/1000000000.0
    secs = math.floor(self.t)
    nsecs = int((self.t - secs)*1000000000.0)
    # if from trackem...
    qw = math.cos(0.5*h)
    qx = 0
    qy = math.sin(0.5*h)
    qz = 0
    pose = [seq,secs,nsecs,x,y,z,qx,qy,qz,qw]
    return pose # [seq,secs,nsecs,x,y,z,qx,qy,qz,qw]

#def pack_pose2d_as_pose(datalist):
#    pass

def pullHeader(atTopLevel):
    """
    input: atTopLevel (dictmsg that is giving Header)
    output: data = Head() (basic class version, not ROS std_msgs one)
    """
    data = Head()
    data.seq = atTopLevel['seq']
    data.stamp.secs = atTopLevel['stamp']['secs']
    data.stamp.nsecs = atTopLevel['stamp']['nsecs']
    return data

#class GeometryMsgsPose():
#    def __init__(self):
#        self.position = GeometryMsgsPoint()
#        self.orientation = GeometryMsgsQuaternion()

class MapMetaData():
    def __init__(self):
        """
        from nav_msgs/MapMetaData.msg:
        
        # This holds basic information about the characteristics of the OccupancyGrid
        time map_load_time # The time at which the map was loaded
        float32 resolution # The map resolution [m/cell]
        uint32 width # Map width [cells]
        uint32 height # Map height [cells]
        geometry_msgs/Pose origin # The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
        """
        self.map_load_time = Stam()
        self.resolution = float()
        self.width = int()
        self.height = int()
        self.origin = [] # GeometryMsgsPose()

def pullPiecePose(atTopLevel):
    """
    input: atTopLevel (dictmsg giving Pose only)
    output: flat list [header.seq, secs, nsecs, x, y, z, qx, qy, qz, qw]
    
    covar = range(0,36) # should give 0 to, not through, 36, a.k.a. 0 through 35 (-- for 36 values)
    pose_json = {'header': pose_json_header, 'child_frame_id': '', 
                'pose': {'pose': {'position': {'x': x, 'y': y, 'z': z},
                                'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw}},
                                'covariance': covar},
                'twist': {'twist' : {'linear': {'x': 0, 'y': 0, 'z': 0},
                                    'angular': {'x': 0, 'y': 0, 'z': 0}} ,
                        'covariance': covar}
    """
    piecepose = [atTopLevel['header']['seq'] ,
                 atTopLevel['header']['stamp']['secs'] ,
                 atTopLevel['header']['stamp']['nsecs'] ,
                 atTopLevel['pose']['pose']['position']['x'] ,
                 atTopLevel['pose']['pose']['position']['y'] ,
                 atTopLevel['pose']['pose']['position']['z'] ,
                 atTopLevel['pose']['pose']['orientation']['x'] ,
                 atTopLevel['pose']['pose']['orientation']['y'] ,
                 atTopLevel['pose']['pose']['orientation']['z'] ,#]
                 atTopLevel['pose']['pose']['orientation']['w'] ] # now also included in output
    return piecepose

def pullPieceMapMetaData(atTopLevel):
    """
    input: atTopLevel (dictmsg that is giving MapMetaData)
    output: data = MapMetaData() (basic class version, not ROS nav_msgs one)
    """
    data = MapMetaData()
    data.map_load_time.secs = atTopLevel['map_load_time']['secs']
    data.map_load_time.nsecs = atTopLevel['map_load_time']['nsecs']
    data.resolution = atTopLevel['resolution']
    data.width = atTopLevel['resolution']
    data.height = atTopLevel['height']
    data.origin = pullPiecePose(atTopLevel['origin'])
    return data

def pullInt8List(atTopLevel):
    data = atTopLevel
    return data
    
def unpack_nav_msgs_OccupancyGrid(dictmsg):
    """
    input: dictmsg
    output: occupancygrid = BlankClass() with .header, .info, .data
            (basic class version, not ROS nav_msgs/OccupancyGrid one)
    
    from nav_msgs/OccupancyGrid.msg:
    
    # This represents a 2-D grid map, in which each cell represents the probability of occupancy.
    Header header
    MapMetaData info #MetaData for the map
    int8[] data # The map data, in row-major order, starting with (0,0).  Occupancy probabilities are in the range [0,100].  Unknown is -1.
    """
    occupancygrid = BlankClass()
    occupancygrid.header = pullHeader(dictmsg['msg']['header'])
    occupancygrid.info = pullPieceMapMetaData(dictmsg['msg']['info'])
    occupancygrid.data = pullInt8list(dictmsg['msg']['data'])
    return occupancygrid

#def pack_nav_msgs_OccupancyGrid(datalist)
#    """
#    
#    """
#    [t,??,listgrid]=datalist
#    occupancygrid_json_header = json_header(t)
#    occupancygrid_json_mapmetadata = json_mapmetadata(??)
#    occupancygrid_json_int8list = json_int8list(listgrid)
#    occupancygrid_json = {'header': occupancygrid_json_header, 'info': occupancygrid_json_mapmetadata, 'data': occupancygrid_json_int8list}
#    return occupancygrid_json

class BlankClass(object): # blank class object allows .___ to be added to this object/class...
    pass

#depstr = 'NOROSLOCAL'

#if (depstr == 'ROSLOCAL'):
    ## because uses "devel/lib/python2.7/site-packages", msgs use dot-hierarchy of dir's(!)
    #import std_msgs.msg
    ##from nav_msgs.msg import Odometry
    ##from geometry_msgs.msg import Twist
    #import nav_msgs.msg
    #import geometry_msgs.msg
    ##import rosaria_msgs.msg #?
    ##import actionlib_msgs.msg #?

#
# an example of ws4py socket comms use, talking to a rosbridge_server on port 9090:
#
if __name__=="__main__":
    # works:
    #from nav_msgs.msg import Odometry
    #loc = Odometry()
    #loc.pose.pose.position.x = 3.10
    #loc.pose.pose.position.y = -1.10
    #print('%r' % loc)
    #print('%s' % loc)

#    from geometry_msgs.msg import Twist
#    fullpose = Twist()
#    fullpose.linear.x = 0.1
#    ...
    fullpose = BlankClass()
    fullpose.linear = BlankClass()
    fullpose.angular = BlankClass()
    fullpose.linear.x = 0.1
    fullpose.linear.y = 0
    fullpose.linear.z = 0
    fullpose.angular.x = 0
    fullpose.angular.y = 0
    fullpose.angular.z = 0
    
    import getConnectionIPaddress as gC
    connection = gC.getConnectionIPaddress()
    #connection = "ws://localhost:9090/" # example string for connecting to rosbridge_server
    #connection = gC.getConnectionIPaddress(3) # this should give back: "ws://localhost:9090/"

    try:
        ws = SubClient(connection)
        # .connect(ROS topic, ROS msg type, data unpacking function (see above))
        ws.connect('/robot0/runtype','std_msgs/Int8',unpack_runtype)
        # sleep a bit to make sure it's been given enough time to open
        sleep(0.20) # may need slightly more time over the network the first time
        # retrieves data if it exists, None if no receipt since last call
        runtype = ws.copy_and_clear_received() # retrieves data if it exists, None if no receipt since last call
        #print("runtype = %r " % runtype)

        #
        # if you were running MobileSim and RosAria, you could use the below command to control the robot's motion:
        #
        ws2 = PubClient(connection) # connection = "ws://localhost:9090/" # example string for connecting to rosbridge_server
        # .connect(ROS topic, ROS msg type, data packing function (see above))
        ws2.connect('/RosAria/cmd_vel','geometry_msgs/Twist',pack_cmdvel)
        #
        # if you were running p3dx in gazebo, you could use this ws2 instead set up control the robot's motion:
        #
        #ws2 = PubClient(connection) # connection = "ws://localhost:9090/" # example string for connecting to rosbridge_server
        # .connect(ROS topic, ROS msg type, data packing function (see above))
        #ws2.connect('/cmd_vel','geometry_msgs/Twist',pack_cmdvel)
        # sleep a bit to make sure it's been given enough time to open
        sleep(0.20) # may need slightly more time over the network the first time
        #sleep(0.05) # need to stay open long enough to connect and get a response
        # then set up and send the command:
        cmdvelcmd = {'linear': {'x': fullpose.linear.x, 'y': fullpose.linear.y, 'z': fullpose.linear.z}, \
                     'angular': {'x': fullpose.angular.x, 'y': fullpose.angular.y, 'z': fullpose.angular.z}}
        ws2.rosconn.send_message(cmdvelcmd) # to send directly in JSON format through ws4py connection interface
        #print("cmdvelcmd sent!\n%r" % cmdvelcmd)
        
        #
        # example code for sending waypoints using packing function to translate datalist to JSON format
        #
#        # Note that the following may only work if you run this python script from the directory in which it resides...
#        #
#        # This is done so you can use the import command on other/separate modules (this is adding it to the beginning like it should've automagically done for you); remember to create an empty __init__.py in the local directory for this to load properly
#        import os
#        import sys # for sys.exit() and sys.path.append()
#        sys.path.append(os.getcwd()) # modify sys.path to include current directory
#        sys.path.append(os.getcwd() + '/../common') # modify sys.path to include ../common directory
#        import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
#        #.connect(ROS topic, ROS msg type, data packing function (see above))
#        ws_waypts_out = ws4pyROS.PubClient(connection)
#        ws_waypts_out.connect('/robot0/waypoint_list','nav_msgs/Path',ws4pyROS.pack_waypoints)
        ws_waypts_out = PubClient(connection)
        # .connect(ROS topic, ROS msg type, data packing function (see above))
        ws_waypts_out.connect('/robot0/waypoint_list','nav_msgs/Path',pack_waypoints)
        sleep(3)
        waypoints_to_send = [ [0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0], [0.0, 0.0] ] # list of lists
        # send waypoint(s), waytype=1 ([x,y] points, no set orientation)
        ws_waypts_out.send_pieces([waypoints_to_send, 1])
        #print("waypoints_to_send sent!\n%r" % waypoints_to_send)

        sleep(3)
    except KeyboardInterrupt:
        ws.close()
        ws2.close()
        ws_waypts_out.close()


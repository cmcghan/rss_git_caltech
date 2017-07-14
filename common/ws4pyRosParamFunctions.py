#! /usr/bin/env python
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

#class ConvertJSON(object):
#    def __init__(self,jsonObjThing):
#        self.__dict__ = loads(str(jsonObjThing))

#class CopyJSON(object):
#    def __init__(self,jsonObj):
#        self.__dict__ = jsonObj

class RosParamServClient(WebSocketClient):
  
    def __init__(self, arg):
        super(RosParamServClient,self).__init__(arg) # super inheritance
        self.dictmsg = None # adding element(s) to base class to hold the data received # raw json msg data in
        #self.rosmsg = None # adding element(s) to base class to hold the data received # raw ros msg data in
        from multiprocessing import Lock # .acquire(block={true,false},timeout={None,int})
        self.jsonMutex = Lock()
        self.servicestr = None
        self.args = None
        self.unpackingfunction = None

    #
    # https://github.com/RobotWebTools/rosbridge_suite/blob/develop/rosapi/scripts/rosapi_node
    # https://github.com/RobotWebTools/rosbridge_suite/blob/groovy-devel/ROSBRIDGE_PROTOCOL.md
    #
    # rosbridge_server parameter services:
    #
    
    # '/rosapi/set_param' -- request.name, request.value
    # '/rosapi/get_param' -- request.name(, request.default ?)
    # '/rosapi/has_param' -- request.name
    # '/rosapi/search_param'  -- request.name
    # '/rosapi/delete_param' -- request.name
    # '/rosapi/get_param_names' -- ...
    def connect(self, servicestr, argname=None, argvalue=None, unpackingfunction=None):
        self.servicestr = servicestr
        
        #self.args = args # needs to be put in JSON format
        if (servicestr == '/rosapi/set_param'):
            if (argname is None):
                print("Error: no argname received for service call: %s." % servicestr)
                sys.exit(0)
            if (argvalue is None):
                print("Error: no argvalue received for service call: %s." % servicestr)
                sys.exit(0)
            self.args = {'name': argname, 'value': str(argvalue)}  # all arguments have to be strings
        elif (servicestr == '/rosapi/get_param') or (servicestr == '/rosapi/has_param') or (servicestr == '/rosapi/search_param') or (servicestr == '/rosapi/delete_param'):
            if (argname is None):
                print("Error: no argname received for service call: %s." % servicestr)
                sys.exit(0)
            self.args = {'name': argname}
        elif (servicestr == '/rosapi/get_param_names') or (servicestr == '/rosapi/topics') or (servicestr == '/rosapi/services'):
            self.args = None
        elif (servicestr is not None) and (argvalue is not None):
            self.args = argvalue
        else:
            print("Error: unknown servicestr %s." % servicestr)
            sys.exit(0)
        
        # unpackingfunction needs to be set, if one is not already given:
        if (unpackingfunction is None):
            if (servicestr == '/rosapi/get_param'):
                self.unpackingfunction = unpack_value
            elif (servicestr == '/rosapi/set_param') or (servicestr == '/rosapi/delete_param'):
                self.unpackingfunction = unpack_result
            elif (servicestr == '/rosapi/has_param'):
                self.unpackingfunction = unpack_exists
            elif (servicestr == '/rosapi/search_param'):
                self.unpackingfunction = unpack_globalname
            elif (servicestr == '/rosapi/get_param_names'):
                self.unpackingfunction = unpack_nameslist
            elif (servicestr == '/rosapi/topics'): # see: http://docs.ros.org/jade/api/rosapi/html/index-msg.html
                self.unpackingfunction = unpack_topics
            elif (servicestr == '/rosapi/services'):
                self.unpackingfunction = unpack_services
        else:
            self.unpackingfunction = unpackingfunction

        try:
            super(RosParamServClient,self).connect() # super inheritance
            self.isOpen = True
        except:
            print("Exception in RosParamServClient: problem connect()ing") # assumes socket.error [Errno 111] connection refused
            self.isOpen = False

    #
    # { "op": "call_service", "service": "_string_", (opt)"args": __list(json)__ }
    # { "op": "service_response", "service": "_string_", (opt)"values": __list(json)__ }
    #
    def opened(self):
        self.dictmsg = None
        #print "DEBUG: Connection opened..."
        msg = {'op': 'call_service', 'service': self.servicestr, 'args': self.args}
        self.send(dumps(msg))
  
    #def close(self): # calls .closed()
    def closed(self, code, reason=None):
        #print code, reason
        #print "DEBUG: %r %r" % (code, reason)
        pass
  
    def received_message(self, m):
        #print "Received:", m # this clears out the string apparently?
        # DEBUG: comment out line below once finished trying out!
        #print("Received: %r" % m) # while this does not -- Received: <ws4py.messaging.TextMessage object at 0x8d7550>
        self.jsonMutex.acquire(True)
        if m.is_text:
            #msg_str = m.data.decode("utf-8")
            #print("Received: %s" % msg_str)
            #dictmsg = loads(msg_str)
            self.dictmsg = loads(str(m)) # still 'unicode' (strings) like this
            #self.dictmsg = loads(m.data.decode("utf-8")) # still 'unicode' (strings) like this
            self.jsonMutex.release()

    def copy_and_clear_received(self):
        self.jsonMutex.acquire(True)
        if (self.dictmsg == None):
            self.jsonMutex.release()
            return None
        else:
#            print("Data: %r" % (self.dictmsg,)) # debug
            unpackeddata = self.unpackingfunction(self.dictmsg)
            self.dictmsg = None
            self.jsonMutex.release()
            return unpackeddata

# get_param (default unpackingfunction)
def unpack_value(dictmsg): # simple string from param server
    value = str(dictmsg['values']['value']) # need to convert message from unicode (string)
    return value

# set_param, delete_param (default unpackingfunction)
def unpack_result(dictmsg): # simple True/False from param server
    result = dictmsg['result']
    return result
    
# has_param (default unpackingfunction)
def unpack_exists(dictmsg): # simple True/False from param server
    #print("unpack_exists = %r" % dictmsg)
    if ('exists' in dictmsg['values']): # then we can read it
        result = dictmsg['values']['exists']
    else: # we likely got some error string like u'Service /rosapi/has_param does not exist' instead of the dict we want
        result = None # should be treated like a "didn't get this get" and force a retry
    return result
    
# search_param (default unpackingfunction)
def unpack_globalname(dictmsg): # simple True/False from param server
    result = dictmsg['values']['global_name']
    return result

# get_param_names (default unpackingfunction)
def unpack_nameslist(dictmsg): # simple list of names / strings from param server
    result = dictmsg['values']['names']
    return result
    
# topics (default unpackingfunction) # see: http://docs.ros.org/jade/api/rosapi/html/index-msg.html
def unpack_topics(dictmsg): # simple list of names / strings from param server OF TOPICS
    #print("DEBUG: unpack_topics = %r" % dictmsg)
    if ('topics' in dictmsg['values']): # then we can read it
        result = dictmsg['values']['topics']
    else: # we likely got some error string like u'Service /rosapi/has_param does not exist' instead of the dict we want
        result = None # should be treated like a "didn't get this get" and force a retry
    return result

# services (default unpackingfunction)
def unpack_services(dictmsg): # simple list of names / strings from param server OF TOPICS
    #print("DEBUG: unpack_services = %r" % dictmsg)
    if ('services' in dictmsg['values']): # then we can read it
        result = dictmsg['values']['services']
    else: # we likely got some error string like u'Service /rosapi/has_param does not exist' instead of the dict we want
        result = None # should be treated like a "didn't get this get" and force a retry
    return result


class BlankClass(object): # blank class object allows .___ to be added to this object/class...
    pass

#
#
#

# changing single-string output directly from string to int right away...
def unpack_testtype(dictmsg): # simple Int32 from param server
    testtype = dictmsg['values']['value'] # need to convert message from unicode (string)
    return int(testtype)

#
# an example of ws4py socket comms use, talking to a rosbridge_server on port 9090:
#
if __name__=="__main__":
    # works:
#    fullpose = BlankClass()
#    fullpose.linear = BlankClass()
#    fullpose.angular = BlankClass()
#    fullpose.linear.x = 0.1
#    fullpose.linear.y = 0
#    fullpose.linear.z = 0
#    fullpose.angular.x = 0
#    fullpose.angular.y = 0
#    fullpose.angular.z = 0

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

    from rss_git_lite.common import getConnectionIPaddress as gC
    connection = gC.getConnectionIPaddress()

    try:
        # run `roscore`, then `roslaunch rosbridge_diff_websocket.launch`
        # then `rosparam set /globals "testtype: 15"`
        # or
        # `rosparam set /globals/testtype 15`
        # also: `rosparam list`
        # full commandline set: rosparam {set,get,list,load,dump,delete}
        # http://wiki.ros.org/rosparam
        
        #
        # '/rosapi/get_param' -- request.name(, request.default?) -- note that get_param call will fail/crash here if /globals/testtype is not on the param server
        #
        # connection = "ws://localhost:9090/" # example string for connecting to rosbridge_server
        ws = RosParamServClient(connection)
        # .connect(ROS topic, service arguments, data unpacking function (see above))
        ws.connect('/rosapi/get_param',"/globals/testtype",None,unpack_testtype)
        sleep(0.20) # may need slightly more time over the network the first time
        testtype = ws.copy_and_clear_received() # retrieves data if it exists, None if no receipt since last call
        print("get_param /globals/testtype = %d" % testtype)
        ws.close() # calls .closed()
        sleep(1)

        #
        # '/rosapi/set_param' -- request.name, request.value -- NOTE: all values are set to string(s)
        #
        ws = RosParamServClient(connection)
        # .connect(ROS topic, service arguments, data unpacking function (see above))
        #ws.connect('/rosapi/set_param',"/globals/testtype2",19,unpack_result)
        ws.connect('/rosapi/set_param',"/globals/testtype2",19)
        sleep(0.20) # may need slightly more time over the network the first time
        testtype = ws.copy_and_clear_received() # retrieves data if it exists, None if no receipt since last call
        print("set_param /globals/testtype2 = %s" % testtype)
        ws.close() # calls .closed()
        sleep(1)
        
        #
        # '/rosapi/get_param' -- request.name(, request.default?) -- checking set_param call...
        #
        # connection = "ws://localhost:9090/" # example string for connecting to rosbridge_server
        ws = RosParamServClient(connection)
        # .connect(ROS topic, service arguments, data unpacking function (see above))
        #ws.connect('/rosapi/get_param',"/globals/testtype2",None,unpack_testtype)
        ws.connect('/rosapi/get_param',"/globals/testtype2")
        sleep(0.20) # may need slightly more time over the network the first time
        testtype = ws.copy_and_clear_received() # retrieves data if it exists, None if no receipt since last call
        print("get_param /globals/testtype2 = %d" % int(testtype))
        ws.close() # calls .closed()
        sleep(1)

        #
        # '/rosapi/has_param' -- request.name
        #
        ws = RosParamServClient(connection)
        # .connect(ROS topic, service arguments, data unpacking function (see above))
        #ws.connect('/rosapi/has_param',"/globals/testtype",None,unpack_exists)
        ws.connect('/rosapi/has_param',"/globals/testtype")
        sleep(0.20) # may need slightly more time over the network the first time
        testtype = ws.copy_and_clear_received() # retrieves data if it exists, None if no receipt since last call
        print("has_param /globals/testtype ? = %s" % testtype)
        ws.close() # calls .closed()
        sleep(1)
        
        #
        # '/rosapi/search_param'  -- request.name
        #
        ws = RosParamServClient(connection)
        # .connect(ROS topic, service arguments, data unpacking function (see above))
        #ws.connect('/rosapi/search_param',"/globals/testtype2",None,unpack_globalname)
        ws.connect('/rosapi/search_param',"/globals/testtype2")
        sleep(0.20) # may need slightly more time over the network the first time
        testtype = ws.copy_and_clear_received() # retrieves data if it exists, None if no receipt since last call
        print("search_param /globals/testtype2 ? = %s" % testtype)
        ws.close() # calls .closed()
        sleep(1)
        
        #
        # '/rosapi/delete_param' -- request.name
        #
        ws = RosParamServClient(connection)
        # .connect(ROS topic, service arguments, data unpacking function (see above))
        #ws.connect('/rosapi/delete_param',"/globals/testtype2",None,unpack_result)
        ws.connect('/rosapi/delete_param',"/globals/testtype2")
        sleep(0.20) # may need slightly more time over the network the first time
        testtype = ws.copy_and_clear_received() # retrieves data if it exists, None if no receipt since last call
        print("delete_param /globals/testtype2 = %s" % testtype)
        ws.close() # calls .closed()
        sleep(1)
        
        #
        # '/rosapi/get_param_names' -- ... -- note that this shows that /globals/testtype2 really was deleted :)
        #
        ws = RosParamServClient(connection)
        # .connect(ROS topic, service arguments, data unpacking function (see above))
        #ws.connect('/rosapi/get_param_names',None,None,unpack_nameslist)
        ws.connect('/rosapi/get_param_names')
        sleep(0.20) # may need slightly more time over the network the first time
        testtype = ws.copy_and_clear_received() # retrieves data if it exists, None if no receipt since last call
        print("get_param_names = %r" % testtype)
        ws.close() # calls .closed()
        sleep(1)
        
#        sleep(3)
    except KeyboardInterrupt:
        ws.close() # calls .closed()

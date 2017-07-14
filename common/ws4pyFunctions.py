# Copyright 2017 by University of Cincinnati
# Copyright 2015-2016 by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite
"""
=== Summary of contents ===
"""

from time import sleep

# -- this file is not executable, must add rss_git_lite to python path prior to import and use!!
#
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
#import sys # for sys.exit() and sys.path.append()
#file_dir = sys.path[0] # *** initialized on program startup, directory of the script used to invoke the python interpreter ***
#sys.path.append(file_dir + '/../..') # modify sys.path to include directory containing rss_git_lite "package"
#print("sys.path = %r\n" % sys.path)

from rss_git_lite.common import ws4pyRosParamFunctions as wsPF
import numpy as np

def receiveInitPositionParamServer(connection):
    """
    input: connection is a string (e.g., 'ws://localhost:9090/')
    
    output: startpt
    """
    startpt = generalRecvParamServer(connection,"/globals/startpt/",['x','y','z'],'block')
    return startpt

def receiveMapOffsetParamServer(connection):
    """
    input: connection is a string (e.g., 'ws://localhost:9090/')
    
    output: mapoffset
    """
    mapoffset = generalRecvParamServer(connection,"/globals/worldmap/mapoffset/",['x','y','z'],'block')
    return mapoffset

def unpack_testservice_srv(dictmsg):
    """
    This is just an example of setting up an "unpacking function" for a
    service call when receiving the response back.
    
    example of expected format of 'dictmsg':
    dictmsg = { ... , \
               'values': 
                   {'useful': True,
                    'points': [{'x': 1.0, 'y': 2.0, 'z': 3.0},{'x': 4.0, 'y': 5.0, 'z': 6.0},...]}

    ROS .srv file should have been set up as:
    # -------------------
    # Request constants:
    # ...
    # -------
    # Request fields:
    std_msgs/String dataFile
    # -------------------
    # Response constants:
    # ...
    # --------
    # Response fields:
    # was a useful set of points given back?
    std_msgs/Bool useful
    # holds points if useful, empty list [] if not
    geometry_msgs/Point[] points
    # -------------------
    """
    dataunpack = ['useful','points']
    result = [None]*len(dataunpack)
    values = dictmsg['values']
    #print("DEBUG: dictmsg['values'] = %s" % str(values))
    if isinstance(values,str):
        print("We got an error: %s" % str(values))
        return result
    else:
        for i in range(0,len(dataunpack)):
            if (dataunpack[i] in values): # then we can read it
                result[i] = values[dataunpack[i]]
                #print("DEBUG: dictmsg['values']['%s']=%r" % (dataunpack[i],result[i])
            else:
                pass # result[i] stays = None
    # convert over 'points' list of dicts [{x: #, y: #, z: #},{x: #, y: #, z: #},...] to list of lists...
    listoflists = []
    for dictpiece in result[1]:
        listoflists.append([dictpiece['x'],dictpiece['y'],dictpiece['z']])
    result[1] = listoflists
    return result

def connectTo_testserviceServer(connection='ws://localhost:9090/',servicestr='/testservice_Solve',argvalues=['data to send','this string not used currently...']):
    """
    This is just an example of setting up a "call" to a service and receiving the response back.
    
    inputs: connection = (default)'ws://localhost:9090/'
            servicestr = {(default)'/testservice_Solve' , '/testserviceServer/get_loggers' , '/testserviceServer/set_logger_level'} # see also: rosservice list
            # rosservice info --> type: testservice/test_msg (here, is a std_msgs/String) , args: datastring
            # rossrv list --> topicstr = {(default)'/testservice/test_msg', '...', '/testservice_srvs/test'} # see also for format: rossrv show /testservice/test_msg -r
            argvalues = [(default)'data to send']

    output: response (raw data that was unpacked using the unpack_testservice_srv() function)
    
    ROS .srv file should have been set up as:
    # -------------------
    # Request constants:
    # ...
    # -------
    # Request fields:
    std_msgs/String dataFile
    # -------------------
    # Response constants:
    # ...
    # --------
    # Response fields:
    # was a useful set of points given back?
    std_msgs/Bool useful
    # holds points if useful, empty list [] if not
    geometry_msgs/Point[] points
    # -------------------
    """
    ws = wsPF.RosParamServClient(connection)
    # Input datastring for testservice <-- datastring is a std_msgs/String in this instance
    datadict = {'dataFile': {'data': argvalues[0]}}
    ws.connect(servicestr, argname=None, argvalue=datadict, unpackingfunction=unpack_testservice_srv) # see above
    sleep(0.02)
    if (ws.isOpen == True):
        response = None
        while (response is None): # because apparently .copy_and_clear_received() might return None if we didn't wait for the response long enough?
            sleep(0.02)
            response = ws.copy_and_clear_received() # retrieves data if it exists, None if no receipt since last call
        ws.close() # calls .closed()
    else: # if RosParamServClient throws an exception
        response = None
    sleep(0.1)
    return response

def checkParamExistsOnServer(connection,paramtopic,blockwait=0.1):
    """
    This is just an example of setting up a "call" to a service and receiving the response back.
    
    inputs: connection is a string (e.g., 'ws://localhost:9090/')
            paramtopic is a string (e.g., "/globals/testtype")
            blockwait is a multiplier on the time to wait in seconds (default=0.1)
            
    algorithm: opens a connection to the ROS parameter server
               check to see if parameter exists on the server
               sleeps for 0.20*blockwait seconds
               checks to see if the connection is open / did not throw an error
                   if no error, copies over received response on parameter status as bool
                          closes connection
                   if errored out, response = None
    
    output: response (should return True or False ; returns None if server is not up)
    """
    ws = wsPF.RosParamServClient(connection)
    # .connect(ROS topic, service arguments, data unpacking function (see above))
    #ws.connect('/rosapi/has_param',"/globals/testtype",None,unpack_exists)
    ws.connect('/rosapi/has_param',paramtopic)
    #sleep(0.20) # may need slightly more time over the network the first time
    sleep(0.20*blockwait)
    if (ws.isOpen == True):
        response = bool(ws.copy_and_clear_received()) # retrieves data if it exists, None if no receipt since last call
        ws.close() # calls .closed()
    else: # if RosParamServClient throws an exception
        response = None
    return response # should return True or False ; returns None if server is not up

def generalRecvTopicsServicesList(connection,servicestr='/rosapi/topics',topicstr=None,blockType='nonblock',blockwait=1.0):
    """
    input: connection (e.g., "ws://localhost:9090/")
           servicestr (default = '/rosapi/topics') is one of: {'/rosapi/topics','/rosapi/services'}
           topicstr (string, default = None) is useful for searching for a particular topic or service in what is returned
           blockType (default='nonblock') is whether to wait/block until receive a response, one of: {'block','nonblock'}
           blockwait (default=1.0) is a multiplier on the time to wait (in loop) between check attempts
               (wait time is 0.20*blockwait seconds; default wait time = 0.20 seconds)
    
    output: data (list OF STRINGS -- CONVERSION MAY BE REQUIRED!!)
    
    example call:
        from rss_git_lite.common import ws4pyFunctions as wsF
        listOfTopics = wsF.generalRecvTopicsServicesList(connection,'/rosapi/services',None,'nonblock')
        # returns a list of strings or None
    
    example call if looking/checking for existence of a specific topic:
        from rss_git_lite.common import ws4pyFunctions as wsF
        doesTopicExist = wsF.generalRecvTopicsServicesList(connection,"/rosapi/topics","/RosAria/cmd_vel",'block')
        # returns one of: {True,False,None}
        
    see: http://docs.ros.org/jade/api/rosapi/html/index-msg.html
    """
    #subnames = list of strings;
    ws = []; data = []
    ws = wsPF.RosParamServClient(connection)
    if (blockType == 'block'):
        while (1):
            ws.connect(servicestr)
            sleep(0.20*blockwait)
            if (ws.isOpen == True):
                checkhold = ws.copy_and_clear_received() # retrieves data if it exists, None if no receipt since last call
                ws.close() # calls .closed()
            else: # if RosParamServClient throws an exception
                checkhold = None
            
            if (checkhold is None): # with return of None, server not up, will block until server is up
                print("Waiting for parameter server, or for %s to become available on parameter server..." % (servicestr,))
            else: # if (checkhold is a list of u'strings'):
                # then see if the requested string is in there
                datahold = [str(checkpiece) for checkpiece in checkhold] # rather than straight-send to convert..() which doesn't work on [u'/rosout', u'/rosout_agg']
                if (topicstr is None): # then the caller wants the entire list
                    data = datahold
                    break
                else: # the caller wants a True/False back
                    for tstr in datahold:
                        if (topicstr == tstr):
                            data = True
                            break
                    data = False
                    break
            sleep(blockwait)
    else: #if (blockType == 'nonblock'):
        # receive data, DO NOT block until topic is available
        ws.connect(servicestr)
        sleep(0.20*blockwait)
        if (ws.isOpen == True):
            checkhold = None
            while (checkhold is None): # because apparently .copy_and_clear_received() might return None if we didn't wait for the response long enough?
                sleep(0.20*blockwait)
                checkhold = ws.copy_and_clear_received() # retrieves data if it exists, None if no receipt since last call
            ws.close() # calls .closed()
        else: # if RosParamServClient throws an exception
            checkhold = None
        
        if (checkhold is None): # with return of None, server not up, will NOT block until server is up
            data = None
        else: # if (checkhold is a list of strings):
            # then see if the requested string is in there
            datahold = [str(checkpiece) for checkpiece in checkhold] # rather than straight-send to convert..() which doesn't work on [u'/rosout', u'/rosout_agg']
            if (topicstr is None): # then the caller wants the entire list
                data = datahold
            else: # the caller wants a True/False back
                for tstr in datahold:
                    if (topicstr == tstr):
                        data = True
                data = False
    #print("DEBUG: topics = %r" % (data,)) # *** DEBUG ***
    #ws.close() # calls .closed()
    #sleep(1)
    sleep(blockwait)
    return data

def generalRecvParamServer(connection,globaltopic,subnames,blockType='nonblock',blockwait=0.5):
    """
    input: connection (e.g., "ws://localhost:9090/")
           globaltopic (string)
           subnames (list of strings, or more complex format -- see below)
           blockType (default='nonblock') is whether to wait/block until receive a response, one of: {'block','nonblock'}
           blockwait (default=0.5) is a multiplier on the time to wait (in loop) between check attempts
               (wait time is 0.20*blockwait seconds in loops; default wait time = 0.10 seconds)
    
    output: data (list, or list of lists, OF STRINGS -- CONVERSION MAY BE REQUIRED!!)
    
    example call:
        from rss_git_lite.common import ws4pyFunctions as wsF
        startpt = wsF.generalRecvParamServer(connection,"/globals/startpt",['x,'y','z'],'nonblock')
        # receives: /globals/startpt/x , /globals/startpt/y , /globals/startpt/z 
        # e.g.: startpt = [ [1.0,2.0,3.0] ]
    
    example for recursive call:
        globaltopic = '/globals/startinfo/'
        subnames = [ ['loc',['x',y']] ]
        from rss_git_lite.common import ws4pyFunctions as wsF
        startpt = wsF.generalRecvParamServer(connection,globaltopic,subnames,'block')
        # receives: /globals/startinfo/loc/x , /globals/startinfo/loc/y
        # e.g.: startpt = [ [1.0,2.0] ]
    """
    #subnames = list of strings;
    ws = []; data = []; wscloselist = [] # wscloselist is for tracking actually-opened connections
    for i in range(len(subnames)):
        if (isinstance(subnames[i],list)): # if there are multiple elements, then must call recursively
            datahold = generalRecvParamServer(connection,globaltopic+subnames[i][0]+'/',subnames[i][1])
            data.append(datahold)
        else:
            ws.append(wsPF.RosParamServClient(connection))
            if (blockType == 'block'):
                while (1):
                    checkhold = checkParamExistsOnServer(connection,globaltopic+subnames[i],blockwait)
                    if (checkhold is None): # with return of None, server not up, will block until server is up
                        print("Waiting for parameter server to come online...")
                    elif (checkhold == True):
                        serviceArg = globaltopic+subnames[i]
                        ws[i].connect('/rosapi/get_param',serviceArg) # .connect(ROS topic, service arguments)
                        wscloselist.append(i)
                        #sleep(0.20) # may need slightly more time over the network the first time
                        datahold = None
                        while (datahold is None): # because apparently .copy_and_clear_received() might return None if we didn't wait for the response long enough?
                            sleep(0.20*blockwait)
                            datahold = ws[i].copy_and_clear_received() # retrieves data if it exists, None if no receipt since last call                        
                        #if (datahold[0] == '"') or (datahold[len(datahold)-1] == '"'): # if datahold is meant to be a string
                        #    datahold = datahold.strip('"') # then it will likely have "" around it from generalSendParamServer(), so strip/remove that if it's there
                        #else: # this was not meant to be a string
                        #    strhold = datahold.strip('[]') # check to see if it's a list
                        #    if (strhold == datahold): # if this isn't a list either (no change detected)
                        #        datahold = float(datahold) # then assume it's a float and convert it
                        #    #else: # otherwise it's a list of some kind
                        #        #pass # so don't attempt to convert it, since it could be in a weird format
                        datahold = convertStringToInternalType(datahold)
                        break
                    else: #if (checkhold == False):
                        print("Waiting for %s to become available on parameter server..." % (globaltopic+subnames[i],))
                    sleep(blockwait)
            else: #if (blockType == 'nonblock'):
                # receive data, DO NOT block until topic is available
                if (checkParamExistsOnServer(connection,globaltopic+subnames[i]) == True,blockwait):
                    serviceArg = globaltopic+subnames[i]
                    ws[i].connect('/rosapi/get_param',serviceArg) # .connect(ROS topic, service arguments)
                    wscloselist.append(i)
                    #sleep(0.20) # may need slightly more time over the network the first time
                    datahold = None
                    while (datahold is None): # because apparently .copy_and_clear_received() might return None if we didn't wait for the response long enough?
                        sleep(0.20*blockwait)
                        datahold = ws[i].copy_and_clear_received() # retrieves data if it exists, None if no receipt since last call
                    #if (datahold[0] == '"') or (datahold[len(datahold)-1] == '"'): # if datahold is meant to be a string
                    #    datahold = datahold.strip('"') # then it will likely have "" around it from generalSendParamServer(), so strip/remove that if it's there
                    #else: # this was not meant to be a string
                    #    strhold = datahold.strip('[]') # check to see if it's a list
                    #    if (strhold == datahold): # if this isn't a list either (no change detected)
                    #        datahold = float(datahold) # then assume it's a float and convert it
                    #    #else: # otherwise it's a list of some kind
                    #        #pass # so don't attempt to convert it, since it could be in a weird format
                    datahold = convertStringToInternalType(datahold)
                else:
                    datahold = None
            data.append(datahold) # add data to list
            #print("DEBUG: get_param %s = %r" % (globaltopic+subnames[i],data[len(data)-1])) # *** DEBUG ***
    for i in wscloselist: # only close connections we actually opened
        ws[i].close() # calls .closed()
    #sleep(1)
    sleep(blockwait)
    return data

def generalSendParamServer(connection,globaltopic,subnames,data):
    """
    input: connection (e.g., "ws://localhost:9090/")
           globaltopic (string)
           subnames (list of strings, or more complex format -- see below)
           data (list, or list of lists)
    
    output: n/a
    
    example call:
        startpt = [ [1.0,2.0,3.0] , ... ]
        from rss_git_lite.common import ws4pyFunctions as wsF
        wsF.generalSendParamServer(connection,"/globals/startpt",['x,'y','z'],startpt[0])
        # gives: /globals/startpt/x , /globals/startpt/y , /globals/startpt/z 
    
    example for recursive call:
        startpt = [ [1.0,2.0] ]
        globaltopic = '/globals/startinfo/'
        subnames = [ ['loc',['x',y']] ]
        from rss_git_lite.common import ws4pyFunctions as wsF
        wsF.generalSendParamServer(connection,globaltopic,subnames,startpt)
        gives: /globals/startinfo/loc/x , /globals/startinfo/loc/y
    """
    #subnames = list of strings;
    ws = []; #data = []
    for i in range(len(subnames)):
        if (isinstance(subnames[i],list)): # if there are multiple elements, then must call recursively
            generalSendParamServer(connection,globaltopic+subnames[i][0]+'/',subnames[i][1],data[i])
        else:
            ws.append(wsPF.RosParamServClient(connection))
            serviceArg = globaltopic+subnames[i]
            if isinstance(data[i],str):
                datapiece = '"%s"' % data[i] # need to show parameter server this is a string, not "just" numbers
            else:
                datapiece = data[i] # can send normally
            ws[i].connect('/rosapi/set_param',serviceArg,datapiece) # .connect(ROS topic, service arguments, data unpacking function (see above))
            #sleep(0.20) # may need slightly more time over the network the first time
            returnType = None
            while (returnType is None):
                sleep(0.02)
                returnType = ws[i].copy_and_clear_received() # retrieves data if it exists, None if no receipt since last call
            #print("DEBUG: set_param %s = %s" % (serviceArg,returnType)) # should be either of: {True,False} # *** DEBUG ***
    for i in range(len(ws)):
        ws[i].close() # calls .closed()
    #sleep(1)
    sleep(0.1)
    return
    
#
# note that these are the primary functions that ought to be used for all (safe) string-to-data conversions from the parameter server:
#

def convertStringToInternalTypeWithStatus(datahold):
    import ast
    try:
        value = ast.literal_eval(datahold)
        status = 0
    except ValueError:
        value = None
        status = -1
    return [value, status]

def convertStringToInternalType(datahold):
    import ast
    try:
        value = ast.literal_eval(datahold)
    except ValueError: # if malformed string...
        value = None
    return value

def convertNpArrayToListofLists(np_array):
    listoflists = []
    for i in range(len(np_array)):
        holdrow = []
        for j in range(len(np_array[i])):
            holdrow.append(np_array[i][j])
        listoflists.append(holdrow)
    return listoflists

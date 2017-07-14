#!/usr/bin/python
# Copyright 2017 by University of Cincinnati
# Copyright 2014-2016 by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite
"""
=== Summary of contents ===
"""

from json import dumps
from ws4py.client.threadedclient import WebSocketClient
from time import sleep
  
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
  
if __name__=="__main__":
    try:
        ws = GetLoggersClient('ws://127.0.0.1:9090/')
        ws.connect()
#       sleep(0.05) # need to stay open long enough to connect and get a response
        sleep(0.20) # the first time takes awhile, so just use this
#        ws.run_forever() # alternately, this forces it to stay open indefinitely until a Ctrl-C is received
    except KeyboardInterrupt:
        ws.close()
#        sleep(1) # if you use .run_forever() then you needs a delay at the end or you get an "Exception in thread WebSocketClient (most likely raised during interpreter shutdown)"

# Copyright 2017 by University of Cincinnati
# Copyright 2014-2016 by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite
"""
=== Summary of contents ===
"""

import socket
import sys

def dns_lookup_or_exit(iphoststr):
    try:
        # gethostbyname() returns IP address when given a hostname
        # note that it will return addr again if given a well-formed IP address
        #      instead, regardless of whether it exists on the network or not
        connection = socket.gethostbyname(iphoststr)
        return connection
    except:
        print("Invalid hostname: '%r'. Exiting." % iphoststr)
        sys.exit(1)

def getConnectionIPaddress(theconn=None):
    if (theconn == None):
        print("What is the IP address for the computer that is hosting the rosbridge server?")
        print("1 = enter IP address (will add 'ws://' and ':9090/' parts automatically)")
        print("2 = enter the hostname (will attempt to resolve, then add 'ws://' and ':9090/' parts)")
        print("3 = 'ws://localhost:9090/' (rosbridge running on same computer)")
        print("4 = 'ws://192.168.56.101:9090/' (Ubuntu VM on laptop)")
        print("5 = 'soong.engr.uc.edu (local -- requires UC VPN)")
        print("6 = 'spacerobotics.rhod.uc.edu (local -- requires UC VPN)")
        theconn = int(raw_input("Input an integer between 1 and 5 inclusive: "))
    if (theconn == 1):
        ipaddystr = str(raw_input("Enter IP address in the form xxx.xxx.xxx.xxx: "))
        if (isValidIPaddress(ipaddystr) == False):
            print("Invalid IP address: '%r'. Exiting." % ipaddystr)
            sys.exit(1)
        else:
            connection = 'ws://' + ipaddystr + ':9090/'
    elif (theconn == 2):
        hoststr = str(raw_input("Enter IP address in the form server.name.with.domain: "))
        connection = 'ws://' + dns_lookup_or_exit(hoststr) + ':9090/' # dns_lookup exits program if not valid
    elif (theconn == 3):
        connection = 'ws://localhost:9090/' # localhost when running command like "ssh -Y USERNAME@localcomputer.xxxx.uc.edu -L 9090:localcomputer.xxxx.uc.edu:9090"
    elif (theconn == 4):
        connection = 'ws://192.168.56.101:9090/'  # Ubuntu VM on laptop
    elif (theconn == 5):
        connection = 'ws://' + dns_lookup_or_exit('soong.engr.uc.edu') + ':9090/' # dns_lookup exits program if not valid
    elif (theconn == 6):
        connection = 'ws://' + dns_lookup_or_exit('spacerobotics.rhod.uc.edu') + ':9090/' # dns_lookup exits program if not valid
    else:
        print("Default connection used -- 'ws://localhost:9090/'")
        connection = 'ws://localhost:9090/'  # localhost when running command like "ssh -Y USERNAME@localcomputer.xxxx.uc.edu -L 9090:localcomputer.xxxx.uc.edu:9090"
    return connection

def isValidIPaddress(ipaddystr):
    if len(ipaddystr)<7: # 1.1.1.1
        return False
    holdsplit = ipaddystr.split('.')
    if len(holdsplit) == 4: # presumably four numbers in here between periods...
        for i in range(0,4): # start at 0, run 4 times(, increment 1)
            if holdsplit[i].isdigit(): # if chunk is a positive integer number (no - or .)
                if (int(holdsplit[i])<256):
                    pass;
                else:
                    print("All numbers in an IP address must be between 0 and 255 inclusive.")
                    return False
            else:
                print("Not a digit: %r" % holdsplit[i].isdigit())
                return False
        # if gets this far, return True
    else:
        print("Not range 4 (not four integer numbers denoted by '.')")
        return False
    
    return True

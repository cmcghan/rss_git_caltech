#! /usr/bin/env python
# Copyright 2017 by University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite

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
sys.path.append(file_dir + '/../../..') # modify sys.path to include directory containing rss_git_lite "package"
#print("sys.path = %r\n" % sys.path)

#print("test_osgetcwd3: now adding '/../../misc' to sys.path so finds test_osgetcwd...")
#sys.path.append(file_dir + '/../../misc') # modify sys.path to include ../common directory
#print("test_osgetcwd3: sys.path = %r\n" % sys.path) # sys.path starts with the directory the file-being-run is currently residing in

print("test_osgetcwd3: now adding '/../../..' to sys.path so starts at rss_git_lite top-level and finds test_osgetcwd...")
sys.path.append(file_dir + '/../../..') # modify sys.path to include directory containing rss_git_lite "package"
print("test_osgetcwd3: sys.path = %r\n" % sys.path) # sys.path starts with the directory the first-file-being-run resides in

print("importing test_osgetcwd...") # a lack of __init__.py still allows import from other directories when modifying the sys.path, interesting
#import test_osgetcwd as t_os
from rss_git_lite.misc import test_osgetcwd as t_os
print("t_os.run_this():")
t_os.run_this()
print("")
print("t_os.run_this2():")
t_os.run_this2()

print("test_osgetcwd3: sys.path = %r\n" % sys.path) # sys.path starts with the directory the file-being-run is currently residing in

print("importing test_osgetcwd2...")
#import test_osgetcwd2 as t_os2
from rss_git_lite.misc import test_osgetcwd2 as t_os2
#from rss_git_lite.common import getConnectionIPaddress as gC # doesn't help if t_os2.trythis() doesn't have the proper from-import-as import line!
print("t_os2.trythis():")
t_os2.trythis() # if test_osgetcwd2 doesn't have any imports for gC, this doesn't work
print("")

print("test_osgetcwd3: sys.path = %r\n" % sys.path) # sys.path starts with the directory the file-being-run is currently residing in

#import getConnectionIPaddress as gC
from rss_git_lite.common import getConnectionIPaddress as gC # need to define "again" in this file's "global" scope, not just t_os2's .trythis() fn
print("gC.dns_lookup_or_exit('google.com') = %r " % gC.dns_lookup_or_exit('google.com'))


# --eof--

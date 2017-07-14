#! /usr/bin/env python
# Copyright 2017 by University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite

# Note that the following may only work if you run this python script from the directory in which it resides...
#
# This is done so you can use the import command on other/separate modules (this is adding it to the beginning like it should've automagically done for you); remember to create an empty __init__.py in the local directory for this to load properly
import os
import sys # for sys.exit() and sys.path.append()
#sys.path.append(os.getcwd()) # modify sys.path to include current directory
#sys.path.append(os.getcwd() + '/../common') # modify sys.path to include ../common directory

def run_this():
    print("sys.path = %r" % sys.path) # sys.path starts with the directory the file-being-run is currently residing in
    print("os.getcwd() = %r" % os.getcwd()) # os.getcwd()  gets directory from which this script was run, NOT the directory this file is in, yikes!
    print("os.getcwd() + '/../common' = %r" % (os.getcwd() + '/../common',))


# Note that the following should work so long as sys.path gives back the
# current directory that the file resides in as its first parameter
# (this should have added it to the beginning of the list automagically
# for you) <-- ***NOPE! THERE ARE PROBLEMS WITH THIS, TOO, SEE: test_osgetcwd3.py ***
#
# This is done so you can use the import command on other/separate
# modules relative to this one but residing in different directories;
# remember to create an empty __init__.py in the local directory for
# things to load properly
#import os # for os.getcwd() if need current working directory that file was called from
#import sys # for sys.exit() and sys.path.append()
#file_dir = sys.path[0] # current directory in which this file resides
#sys.path.append(file_dir + '/../common') # modify sys.path to include ../common directory
#print("sys.path = %r" % sys.path) # sys.path starts with the directory the file-being-run is currently residing in

def run_this2():
    file_dir = sys.path[0] # current directory in which this file resides
    sys.path.append(file_dir + '/../common') # modify sys.path to include ../common directory
    print("sys.path with '/../common' added to end = %r" % sys.path) # sys.path starts with the directory the file-being-run is currently residing in

if __name__ == '__main__':
    run_this()
    print("")
    run_this2()
    print("")
    print("sys.path in main: %r" % sys.path)
    
# --eof--

# Copyright 2017-2018 by University of Cincinnati
# Copyright 2015-2016 by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite
"""
=== Summary of contents ===
"""

# Note that the following may only work if you run this python script from the directory in which it resides...
#
# This is done so you can use the import command on other/separate modules (this is adding it to the beginning like it should've automagically done for you); remember to create an empty __init__.py in the local directory for this to load properly
import os
import sys # for sys.exit() and sys.path.append()
#sys.path.append(os.getcwd()) # modify sys.path to include current directory
#sys.path.append(os.getcwd() + '/../common') # modify sys.path to include ../common directory

def readArgvpieces(sysargv,argvdefaults):
    """
    sysargv = sys.argv (values from commandline call)
    argvdefaults = list of values (length determines number of input parmeters to look for
    returns argvpieces (default values from argvpieces if 'def' in for a value, or missing argument) ('None' is not translated to None)
    example call:
        [var1,
         var2] = readArgvpieces(sys.argv,['test1',2])
    """
    argvpieces = argvdefaults
    argvend = len(argvdefaults)+1
    for i in range(1,argvend):
        if len(sys.argv) > i: # if there's another argument waiting within checked range
            if (sys.argv[i] != 'def'): # not requesting default value, so overwrite
                argvpieces[i-1] = sys.argv[i]
    if len(sys.argv) > argvend:
        print("Warning, too many arguments. Arguments beyond %d ignored." % (argvend-1,) )

    return argvpieces

def readstateDataEstOrRealandestChannelsFromFile(filename='default_estorreal.txt'):
    """
    inputs: filename (string) (default='default_estorreal.txt')
    outputs: layernames (list of strings) (default=['deliberative','habitual','reflexive'])
             stateDataEstOrRealList (list of strings) (strings are = {'Est','Real'}
             estChannelsList (list of strings) (strings are = {'manysingles','combined'})
    example file:
        deliberative Est combined
        habitual Est manysingles
        reflexive Real combined
    example call:
        import fileFunctions as fF
        [layernames,stateDataEstOrRealList,estChannelsList] = fF.readstateDataEstOrRealandestChannelsFromFile('default_estorreal.txt')
        #layernames=['deliberative','habitual','reflexive'] or similar
    """
    # open file
    f = open(filename,'r')
    layernames = []
    stateDataEstOrRealList = []
    estChannelsList = []
    fstr = f.readline()
    while (len(fstr) != 0):
        fstrlist = fstr.split()
        flist = [str(namestr) for namestr in fstrlist] # one polytope (set of indices) per line
        layernames.append(flist[0])
        stateDataEstOrRealList.append(flist[1])
        estChannelsList.append(flist[2])
        fstr = f.readline()
    f.close()
    
    # send data back
    return [layernames,stateDataEstOrRealList,estChannelsList]

#
# this code originally pulled out of csvmapFromPolytopes.py
#

def readPolytopesFromFile(filename):
    """
    read from file
    input: filename = string (name of file)
    file format:
    ------------
    HEADER:
    units is one of: {'feet','meters','inches','centimeters'}
    dim is a: integer (dimension of point in space (e.g., 2 for [x,y]))
    MAIN DATA:
    polytope_list = [ [[x11,y11],...,[x1n,y1n]], ...] (one polytope per line, numbers delineated by spaces)
    ------------
    returns: [polytope_list, units, scale, dim]
    """
    #polytope_list = None; units = None; scale = None; dim = 2 # temporary line
    
    # open file
    f = open(filename,'r')
    polytope_list = []
    unitsstr = f.readline(); unitsstr=unitsstr.split(); units = unitsstr[0] # what do if == None? # .split() removes leading and trailing whitespace :)
    dimstr = f.readline(); dim = int(dimstr) # what do if == None?
    fstr = f.readline()
    while (len(fstr) != 0):
        fstrlist = fstr.split()
        flist = [float(numstr) for numstr in fstrlist] # one polytope (set of indices) per line
        polytope1 = [[flist[j] for j in range(i,i+dim)] for i in range(0,len(flist),dim)]
        polytope_list.append(polytope1)
        fstr = f.readline()
    f.close()
    
    # send data back
    print(units)
    return [polytope_list, units, dim]

def printPolytopesToFile(filename,polytope_list=None,units='meters',dim=2):
    """
    write to file
    input: filename = string (name of file)
           polytope_list
           units (default = 'meters')
           dim (default = 2)
    file format:
    ------------
    HEADER:
    units is one of: {'feet','meters','inches','centimeters'}
    dim is a: integer (dimension of point in space (e.g., 2 for [x,y]))
    MAIN DATA:
    polytope_list = [ [[x11,y11],...,[x1n,y1n]], ...] (one polytope per line, numbers delineated by spaces)
    ------------
    returns: n/a
    """
    if (polytope_list == None):
        from writeVobjsAsArMapFile import defaultV
        [V,boundaryedges] = defaultV()
        polytope_list = V
        dim = len(V[0][0])
    
    f = open(filename,'w')
    headstr = '%s\n%d\n' % (units,dim)
    f.write(headstr)
    for polytope1 in polytope_list:
        fstr = ''
        for indexpt in polytope1: # for each index point in polytope, [[p1],[p2],...,[pn]]
            for num in indexpt: # for [px1,px2,...,pxm] in index point, m = dim
                fstr = fstr + ' ' + str(num)
            fstr = fstr + '  ' # 1+2 spaces between indices in a polytope
        fstr = fstr + '\n'
        f.write(fstr)
    f.close()

def readCsvparamsFromFile(filename="default_csvparams.txt"):
    """
    read from file
    input: filename = string (name of file)
    file format:
    ------------
    MAIN DATA:
    resolution (width, height in pixels (two values) =or= meters-per-pixel (one value))
    size (upper-left x, upper-left y, lower-right x, lower-right y in units)
    units (default = 'meters')
    ------------
    returns: [resolution (list), size (list), units (string), scale (int)]
    """
    # open file
    f = open(filename,'r')
    
    fstr = f.readline()
    fstrlist = fstr.split()
    flist = [float(numstr) for numstr in fstrlist] # width,height in pixels (two values), =or= meters-per-pixel (one value)
    resolution = flist
    
    fstr = f.readline()
    fstrlist = fstr.split()
    flist = [float(numstr) for numstr in fstrlist] # upper-left x, upper-left y, lower-right-x, lower-right-y
    size = flist

    unitsstr = f.readline(); unitsstr=unitsstr.split(); units = unitsstr[0] # what do if == None? # .split() removes leading and trailing whitespace :)

    f.close()
    
    # send data back
    return [resolution,size,units]
    
def readBoundaryedgesFromFile(filename):
    """
    read from file
    input: filename = string (name of file)
    file format:
    ------------
    HEADER:
    units is one of: {'feet','meters','inches','centimeters'}
    dim is a: integer (dimension of point in space (e.g., 2 for [x,y]))
    MAIN DATA:
    boundaryedges = [[x11,y11],...,[x1n,y1n]] (one polytope on a single line, numbers delineated by spaces)
    ------------
    returns: [units, dim, boundaryedges]
    """    
    # open file
    f = open(filename,'r')

    unitsstr = f.readline(); unitsstr=unitsstr.split(); units = unitsstr[0] # what do if == None? # .split() removes leading and trailing whitespace :)
    dimstr = f.readline(); dim = int(dimstr) # what do if == None?

    fstr = f.readline()
    fstrlist = fstr.split()
    flist = [float(numstr) for numstr in fstrlist] # read in all numbers
    polytope1 = [[flist[j] for j in range(i,i+dim)] for i in range(0,len(flist),dim)] # group all numbers
    boundaryedges = polytope1

    f.close()
    
    # send data back
    return [units, dim, boundaryedges]

def readPIDGainsFromFile(filename):
    """
    read from file
    input: filename = string (name of file)
    file format:
    ------------
    MAIN DATA:
    PIDgains = [P, I, D] (gains all on one line, delineated by spaces)
    ------------
    returns: PIDgains
    """    
    # open file
    f = open(filename,'r')

    fstr = f.readline()
    fstrlist = fstr.split() # use spaces to cut parts up
    PIDgains = [float(numstr) for numstr in fstrlist] # all gains on one line, will autoremove spaces and such as part of conversion process
    
    f.close()

    return PIDgains

def readMaxlinangratesFromFile(filename):
    """
    read from file
    input: filename = string (name of file)
    file format:
    ------------
    HEADER:
    (single comment line)
    MAIN DATA:
    maxlinangrates = [[maxlinrate1, maxangrate1],[maxlinrate2, maxangrate2]] (gains all on one line, delineated by spaces)
    ------------
    returns: maxlinangrates
    """    
    # open file
    f = open(filename,'r')

    fstr = f.readline() # read in comment line (1st line of file)
    fstr = f.readline() # then immediately overwrite with next line of data (2nd line of file)
    fstrlist = fstr.split() # use spaces to cut parts up
    flist = [float(numstr) for numstr in fstrlist] # all gains on one line, will autoremove spaces and such as part of conversion process
    dim = 2 # [[maxlinrate,maxangrate],[maxlinrate,maxangrate]] <-- two for each set [[dim2],[dim2]]
    maxlinangrates = [[flist[j] for j in range(i,i+dim)] for i in range(0,len(flist),dim)] # group all numbers
    
    f.close()

    return maxlinangrates

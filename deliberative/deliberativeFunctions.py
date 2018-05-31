# Copyright 2017-2018 by University of Cincinnati
# Copyright 2014-2016 by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite
"""
=== Summary of contents ===
"""

import math

def calculateSomeConstraint(curwaypt,x1,y1,some_constraintCoeff):
    """
    This is a simple example function.
    This function calculates a constraint based on / proportional to the
        distance between the curwaypt and point (x1,y1)
    input: curwaypt is a list[2]
           x1,y1 are float
           some_constraintCoeff is float
    output: someConstraint (float)
    example call:
        import deliberativeFunctions as dF
        someConstraint = dF.calculateSomeConstraint(curwaypt,x1,y1,some_constraintCoeff)
    """
    #calculate distance between current location and next waypoint
    disttry = math.sqrt( (curwaypt[0]-x1)**2 + (curwaypt[1]-y1)**2 )
    someConstraint = disttry * some_constraintCoeff # calculate some_constraint
    return someConstraint
    
def relaxSomeConstraint(someConstraint,some_constraintCoeff,relaxBy):
    """
    This is a simple example function.
    This function relaxes a constraint and constraint-coefficient by a
        the given multiple value (proportional relaxation).
    input: someConstraint,some_constraintCoeff,relaxBy are float
    output: [someConstraint,some_constraintCoeff] (float)
    example call:
        import deliberativeFunctions as dF
        [someConstraint,some_constraintCoeff] = dF.relaxSomeConstraint(someConstraint,some_constraintCoeff,relaxBy)
    """    
    someConstraint = someConstraint * relaxBy # relax some_constraint
    some_constraintCoeff = some_constraintCoeff * relaxBy # lower some_coefficient
    return [someConstraint,some_constraintCoeff]

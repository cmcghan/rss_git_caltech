# Copyright 2017-2018 by University of Cincinnati
# Copyright 2015-2016 by California Institute of Technology
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/rss_git_lite
"""
=== Summary of contents ===
"""

#
# this code was originally written to work with trackem_FASTER_points_to_position_orientation.py
# and modified+extended further thereafter (--to something that should actually work!)
#

import sys

#import math
# from math import *
from math import sin
from math import cos
from math import atan2
from math import sqrt
from math import fabs
from math import acos
from math import pi

def convertQuatToHead(quat):
    """
    quat: [qx, qy, qz, qw]  or [q1, q2, q3, q0] or [q2,q3,q4,q1]
    head: [h, p, r]         or [z, y, x]        or [th1, th2, th3] (radians)
          #this code (h = y)? or  gen. (h = z)   or NASA pdf (see below)
          this code (h = z) or  gen. (h = z)   or NASA pdf (see below)
    in: [qx,qy,qz,qw]
    out: [r,p,h]
    example call:
        import angleTransforms as aT
        [r,p,h] = aT.convertQuadToHead([qx,qy,qz,qw])
    """
    #            ROS  gen. NASA
    qx = quat[0] #qx , q1 , q2
    qy = quat[1] #qy , q2 , q3
    qz = quat[2] #qz , q3 , q4
    qw = quat[3] #qw , q0 , q1

    # the sequence for many ground-based vehicles coordinate system transformations is the following:
    # -- an axis rotation sequence of 3-2-1 (Z-Y-X) --
    #
    # this is sequence (10), as given on page 25 of
    # http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770024290.pdf
    #
    # z is heading-CCW-from-above, y is pitch-nose-down,
    # and x is roll left-to-right up-and-over
    # assuming axes are as follows (for rovers):
    # x is out the front, y is to the left, and z is straight-up
    #
    # for planes, axes may be as follows, instead:
    # z may be pointed -down- from the belly, x out the nose,
    # and y out the -right- wing
    #
    # another helpful website is:
    # http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
    # http://www.euclideanspace.com/maths/standards/index.htm
    # 
    # the links above were found/followed from:
    # http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    # page 25 of the nasa document defines the angles to be:
    # h = atan2(m21, m11)              # theta1 (Z) or  heading / yaw
    # p = atan2(-m31, sqrt(1-m31*m31)) # theta2 (Y) or attitude / pitch
    # r = atan2(m32, m33)              # theta3 (X) or     bank / roll
    # from the quaternion matrix at the nasa document at page 10, we get:
    # qw,qx,qy,qz <-- q1,q2,q3,q4
    # h = atan2( 2*(q2*q3+q1*q4), q1*q1+q2*q2-q3*q3-q4*q4 )
    # p = atan2(-2*(q2*q4-q1*q3), sqrt(1-4*(q2*q4-q1*q3)*(q2*q4-q1*q3)) )
    # r = atan2( 2*(q3*q4+q1*q2), q1*q1-q2*q2-q3*q3+q4*q4 )
    h = atan2( 2*(qx*qy+qw*qz), qw*qw+qx*qx-qy*qy-qz*qz ) # latter == 1 - 2*(qy*qy+qz*qz) == 2*(qw*qw+qx*qx) - 1
    p = atan2(-2*(qx*qz-qw*qy), sqrt(1-4*(qx*qz-qw*qy)*(qx*qz-qw*qy)) )
    r = atan2( 2*(qy*qz+qw*qx), qw*qw-qx*qx-qy*qy+qz*qz ) # latter == 1 - 2*(qx*qx+qy*qy) == 2*(qw*qw+qz*qz) - 1
    # these are returned in radians
    return [r,p,h]

def convertHeadToQuat(head):
    """
    quat: [qx, qy, qz, qw]  or [q1, q2, q3, q0] or [q2, q3, q4, q1]
    head: [h, p, r]         or [z, y, x]        or [th1, th2, th3] (radians)
          #this code (h = y) or  gen. (h = z)   or NASA pdf (see below)
          this code (h = z) or  gen. (h = z)   or NASA pdf (see below)
    in: [r,p,h]
    out: [qx,qy,qz,qw]
    example call:
        import angleTransforms as aT
        [qx,qy,qz,qw] = aT.convertHeadToQuad([r,p,h])
    """
    if (len(head) == 1):
        h = head[0]     # theta1 (Z) or  heading / yaw
    if (len(head) == 3): # 1, 2, 5, 6, 9, 10 -- need same as 10 but swap qz qy (q3 q4) --> 2,5,10 have same sign etc. on qw --> could be 5 as y-x-z maybe?
        h = head[2]     # theta1 (Z) or  heading / yaw
        p = head[1] # theta2 (Y) or attitude / pitch
        r = head[0] # theta3 (X) or     bank / roll
        qw =  sin(0.5*h)*sin(0.5*p)*sin(0.5*r) + cos(0.5*h)*cos(0.5*p)*cos(0.5*r)
        qx = -sin(0.5*h)*sin(0.5*p)*cos(0.5*r) + sin(0.5*r)*cos(0.5*h)*cos(0.5*p)
        qy =  sin(0.5*h)*sin(0.5*r)*cos(0.5*p) + sin(0.5*p)*cos(0.5*h)*cos(0.5*r)
        qz =  sin(0.5*h)*cos(0.5*p)*cos(0.5*r) - sin(0.5*p)*sin(0.5*r)*cos(0.5*h)
        
    else:
        #p = 0
        #r = 0
        qw = cos(0.5*h)
        qx = 0
        qy = 0 #sin(0.5*h)
        qz = sin(0.5*h) #0

    # the sequence for many ground-based vehicles coordinate system transformations is the following:
    # -- an axis rotation sequence of 3-2-1 (Z-Y-X) --
    #
    # this is sequence (10), as given on page 25 of
    # http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770024290.pdf
    #
    # z is heading-CCW-from-above, y is pitch-nose-down,
    # and x is roll left-to-right up-and-over
    # assuming axes are as follows (for rovers):
    # x is out the front, y is to the left, and z is straight-up
    #
    # for planes, axes may be as follows, instead:
    # z may be pointed -down- from the belly, x out the nose,
    # and y out the -right- wing
    #
    # another helpful website is:
    # http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
    # http://www.euclideanspace.com/maths/standards/index.htm
    # 
    # the links above were found/followed from:
    # http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    # page 25 in the nasa document defines translation from euler 3-2-1
    # to quaternion as follows:
    # where (q1) -> (qw) and (q2,q3,q4) -> (qx,qy,qz):
    # q1 =  sin(0.5*th1)*sin(0.5*th2)*sin(0.5*th3) + cos(0.5*th1)*cos(0.5*th2)*cos(0.5*th3)
    # q2 = -sin(0.5*th1)*sin(0.5*th2)*cos(0.5*th3) + sin(0.5*th3)*cos(0.5*th1)*cos(0.5*th2)
    # q3 =  sin(0.5*th1)*sin(0.5*th3)*cos(0.5*th2) + sin(0.5*th2)*cos(0.5*th1)*cos(0.5*th3)
    # q4 =  sin(0.5*th1)*cos(0.5*th2)*cos(0.5*th3) - sin(0.5*th2)*sin(0.5*th3)*cos(0.5*th1)
    #
    #qw =  sin(0.5*h)*sin(0.5*p)*sin(0.5*r) + cos(0.5*h)*cos(0.5*p)*cos(0.5*r)
    #qx = -sin(0.5*h)*sin(0.5*p)*cos(0.5*r) + sin(0.5*r)*cos(0.5*h)*cos(0.5*p)
    #qy =  sin(0.5*h)*sin(0.5*r)*cos(0.5*p) + sin(0.5*p)*cos(0.5*h)*cos(0.5*r)
    #qz =  sin(0.5*h)*cos(0.5*p)*cos(0.5*r) - sin(0.5*p)*sin(0.5*r)*cos(0.5*h)

#    # these are returned in qw,qx,qy,qz order
#    return qw,qx,qy,qz
    # these are returned in qx,qy,qz,qw order
    return [qx,qy,qz,qw]

def reRangeAng(ang,low,high):
    """
    input: ang (angle in radians)
           low (radians)
           high (radians)
    algorithm: re-range the angle (-2pi if ang>high, +2pi if ang<low)
    output: ang (angle in radians, in [low,high])
    
    NOTE: assumes that lo < hi
    """
    while True:
        if ang > high:
            ang = ang - (2 * pi)
        elif ang < low:
            ang = ang + (2 * pi)
        else:
            break
    return ang

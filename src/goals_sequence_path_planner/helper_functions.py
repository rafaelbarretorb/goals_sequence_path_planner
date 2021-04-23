#!/usr/bin/env python

import math


PI = math.pi

def dist(p1, p2):    
    """ Class method for compute the distance between two points.
    Args:
        p1: Point 1 tuple.
        p2: Poinf 2 tuple.
    Returns:
        The distance between two points the in cartesian plan.
    """
    return math.sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def aim_to_point(current_x, current_y, target_x, target_y):
    """ Return the yaw angle of a current point (heading) pointed 
    in the direction of a next point. """

    y2 = target_y
    y1 = current_y

    x2 = target_x
    x1 = current_x

    yaw = math.atan2((y2 - y1), (x2 - x1)) # method returns a numeric value between -PI and PI

    return yaw

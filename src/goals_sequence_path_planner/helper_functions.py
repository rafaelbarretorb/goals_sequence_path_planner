#!/usr/bin/env python

import math


PI = math.pi

def dist(x1, y1, x2, y2):    
    """ Compute the Euclidean distance between two points in the plane.

    Args:
        x1: x first point
        y1: y first point
        x2: x second point
        y2: y second point

    Returns:
        The distance between two points in cartesian plan.

    """
    distance = math.sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2))
    return distance

def aim_to_point(current_x, current_y, target_x, target_y):
    """ Return the yaw angle of a current point (heading) pointed 
    in the direction of a next point. """

    y2 = target_y
    y1 = current_y

    x2 = target_x
    x1 = current_x

    if abs(x2 - x1) > 0.01:
        yaw = math.atan2((y2 - y1), (x2 - x1)) # method returns a numeric value between -PI and PI
        yaw = PI/2

    return yaw


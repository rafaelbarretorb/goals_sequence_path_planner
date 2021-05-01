#!/usr/bin/env python

import math

PI = math.pi


def distance(p1, p2):    
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def aim_to_point(current_x, current_y, target_x, target_y):
    """ Return the yaw angle of a current point (heading) pointed 
    in the direction of a next point. """

    y2 = target_y
    y1 = current_y

    x2 = target_x
    x1 = current_x

    yaw = math.atan2((y2 - y1), (x2 - x1)) # method returns a numeric value between -PI and PI

    return yaw

#!/usr/bin/env python

import math


PI = math.pi

def dist(x1, y1, x2, y2):    
    """ Class method for compute the Euclidean distance between two points in the plane.

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


def aim_to_next_position(path_x, path_y, goal_yaw):
    """ Return the list of orientation of each path position.The orientation of a given position aims to the next position of the path."""

    yaw_list = list()
    for i in range(len(path_x)-1):
        y2 = path_y[i+1]
        y1 = path_y[i]

        x2 = path_x[i+1]
        x1 = path_x[i]


        #yaw_radian = np.arctan2((y2 - y1)/(x2 - x1))
        if abs(x2 - x1) > 0.001:
            #yaw_radian = np.arctan(abs(y2 - y1)/abs(x2 - x1)) # angle in radians between 0 to PI/2
            yaw_radian = math.atan2((y2 - y1), (x2 - x1))
        else:
            yaw_radian = PI/2

        yaw = (180/PI)*yaw_radian # convert to degrees
        yaw_list.append(yaw)

    yaw_list.append(goal_yaw)

    return yaw_list 

def aim_to_point(current_x, current_y, target_x, target_y):
    """ Return the yaw angle of a current point (heading) pointed 
    in the direction of a next point. """

    y2 = target_y
    y1 = target_y

    x2 = target_x
    x1 = current_x

    if abs(x2 - x1) > 0.01:
        yaw_rad = math.atan2((y2 - y1), (x2 - x1)) # angle in radians between 0 to PI/2
    else:
        yaw_rad = PI/2

    yaw = (180/PI)*yaw_rad # convert to degrees

    # Correct angle quadrant
    if (y2 > y1) and (x2 < x1):
        yaw = 180 - yaw
    elif (y2 < y1) and (x2 < x1):
        yaw = 180 + yaw
    elif (y2 < y1) and (x2 > x1):
        yaw = 360 -  yaw

    return math.radians(yaw)

def position_to_pose(path, orientation_list):
    """ Return an array of the n poses
        [[x1,y1,theta1], [x2,y2,theta2], ..., [xn,yn,thetan]]
    """
    waypoints = list()
    if len(path) == len(orientation_list):
        for i in range(path):
            pose = [path[i][0],path[i][1],orientation_list[i]]
            waypoints.append(pose)

    waypoints_arr = np.array(waypoints)

    return waypoints_arr

def get_arrows(path_x, path_y, orientation_list, arrow_length=0.1):
    """ Return the positions of the head of arrows.
        This function is used for plotting the waypoints.
    """

    endx_list = list()
    endy_list = list()

    for i in range(len(path_x)):

        endx = arrow_length*math.cos(math.radians(orientation_list[i]))
        endy = arrow_length*math.sin(math.radians(orientation_list[i]))

        endx_list.append(endx)
        endy_list.append(endy)

    x_end_arr = np.array(endx_list)
    y_end_arr = np.array(endy_list)

    return x_end_arr, y_end_arr

def get_arrow_pose(x, y, theta, arrow_length=0.25):
    endx = arrow_length*math.cos(math.radians(theta))
    endy = arrow_length*math.sin(math.radians(theta))

    return endx, endy
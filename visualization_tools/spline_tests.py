#!/usr/bin/env python

from scipy import interpolate
import scipy.interpolate as si

from world import make_world
from rrt_star import RRT_Star

from orientation_filter import aim_to_next_position, get_arrows, get_arrow_pose


import numpy as np
import matplotlib.pyplot as plt
import pylab
import math

import time # delay

PI = math.pi

def dist(p1,p2):    
    """ Class method for compute the distance between two points.

    Args:
        p1: Point 1 tuple.
        p2: Poinf 2 tuple.

    Returns:
        The distance between two points the in cartesian plan.

    """
    distance = math.sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))
    return distance

def main():

    r = 2.0
    scatter_area = PI*r*r

    #####  TESTS !!!!!!!!!!!!!!!!!!!!!!!!  #################
    
    # T1: OK
    # Start Left and Goal Right
    initial_pose = (0.0, -2.0, 90.0)
    goal_pose = (0.0, 2.0, 0.0)

    # T2: OK
    # Start Right and Goal Left
    #initial_pose = (0.0, -3.0, 90.0)
    #goal_pose = (0.0, 2.0, 150.0)


    # T3: OK
    # 
    #initial_pose = (-3.0,-3.0, 45.0)
    #goal_pose = (1.0,1.0, 0.0)

    # T4: ok
    #initial_pose = (-3.0,-3.0, 45.0)
    #goal_pose = (1.0,1.0, 120.0)

    # T5: OK
    # 
    #initial_pose = [-2.0, 2.0, -45.0]
    #goal_pose = [2.0, -2.0, 0.0]

    # T6: OK

    #initial_pose = [-2.0, 2.0, -45.0]
    #goal_pose = [2.0, -2.0, -90.0]

    # T7: OK
    # 
    #initial_pose = [2.0, 2.0, 225.0]
    #goal_pose = [-2.0, -2.0, 160.0]



    plt.figure(figsize=(10,5))


    # Plot initial and goal pose
    initial_endx, initial_endy = get_arrow_pose(initial_pose[0], initial_pose[1], initial_pose[2], arrow_length=0.2)
    goal_endx, goal_endy = get_arrow_pose(goal_pose[0], goal_pose[1], goal_pose[2], arrow_length=0.2)


    # NEXT GOAL
    x_next_goal = goal_pose[0]+2
    y_next_goal = goal_pose[1]+2


    R_min = 1.0

    new_start = find_new_start(initial_pose, goal_pose, R_min)

    new_start_endx, new_start_endy = get_arrow_pose(new_start[0], new_start[1], initial_pose[2], arrow_length=0.2)

    new_start = [new_start[0], new_start[1],initial_pose[2]]

    s_rx, s_ry, s_rx_c, s_ry_c, s_lx, s_ly, s_lx_c, s_ly_c = plot_circle(new_start[0], new_start[1], new_start[2], R_min)
    g_rx, g_ry, g_rx_c, g_ry_c, g_lx, g_ly, g_lx_c, g_ly_c = plot_circle(goal_pose[0], goal_pose[1], goal_pose[2], R_min)

    start_circle, goal_circle = pick_the_cicle(new_start[2], goal_pose[2])

    if start_circle == "RIGHT":
        xc_s = s_rx_c
        yc_s = s_ry_c
        xc_g = g_lx_c
        yc_g = g_ly_c

        point_intersection = [(s_rx_c + g_lx_c)/2, (s_ry_c+g_ly_c)/2]
        if abs(s_rx_c - g_lx_c) > 0.01:
            #theta = math.atan(abs(s_ry_c - g_ly_c)/abs(s_rx_c - g_lx_c))
            theta = math.atan((s_ry_c - g_ly_c)/(s_rx_c - g_lx_c))
        else:
            theta = PI/2

    else:
        xc_s = s_lx_c
        yc_s = s_ly_c
        xc_g = g_rx_c
        yc_g = g_ry_c

        point_intersection = [(s_lx_c+g_rx_c)/2, (s_ly_c+g_ry_c)/2]
        if abs(s_lx_c - g_rx_c) > 0.01:
            #theta = math.atan(abs(s_ly_c - g_ry_c)/abs(s_lx_c - g_rx_c))
            theta = math.atan((s_ly_c - g_ry_c)/(s_lx_c - g_rx_c))
        else:
            theta = PI/2




    ################################   ################################
    
    xc_s, yc_s = compute_center(new_start[0], new_start[1], new_start[2], R_min, start_circle) 
    xc_g, yc_g = compute_center(goal_pose[0], goal_pose[1], goal_pose[2], R_min, goal_circle)


    
    s_x_list, s_y_list = circular_sector(new_start[0], new_start[1], new_start[2], R_min, start_circle, "start", point_intersection) 


    g_x_list, g_y_list = circular_sector(goal_pose[0], goal_pose[1], goal_pose[2], R_min, goal_circle, "goal", point_intersection)
    

    count = 1

    ################################   ################################


    l_x = [[initial_pose[0]],[new_start[0]], s_x_list, g_x_list[::]]
    l_y = [[initial_pose[1]],[new_start[1]], s_y_list, g_y_list[::]]

    #flatten_x = lambda l_x: [item for sublist in l_x for item in sublist]
    #flatten_y = lambda l_y: [item for sublist in l_y for item in sublist]

    flatten_x = list()
    flatten_y = list()

    flatten_x.append(0.0)
    flatten_x.append(0.0)

    flatten_y.append(-4.0)
    flatten_y.append(-3.0)

    for sublist in l_x:
        for item in sublist:
            flatten_x.append(item)

    for sublist in l_y:
        for item in sublist:
            flatten_y.append(item)

    points = list()

    for i in range(len(flatten_x)):
        t = [flatten_x[i], flatten_y[i]]
        points.append(t)


    data = np.array(points)
    print data


    #x, y = zip(*points2)
    #tck,u = interpolate.splprep(data.transpose(), s=0)
    tck,u = interpolate.splprep(data[::2].transpose(), k=3)
    unew = np.arange(0, 1.05, 0.05)
    out = interpolate.splev(unew, tck)

    ax1 = plt.subplot(121)

    #plt.plot(out[0], out[1], color='orange')
    #plt.scatter(data[:,0], data[:,1])
    plt.scatter(out[0], out[1], color='orange')

    plt.xlim(-5, 5)
    plt.ylim(-5, 5)

    ####################################

    ax2 = plt.subplot(122)



    #plt.plot(cv[:,0],cv[:,1], 'o-', label='Control Points')


    p = bspline(data,n=20,degree=3)
    x,y = p.T

    f2 = interpolate.interp1d(x, y, kind='cubic')


    plt.scatter(x,y, color='orange')
    #plt.scatter(x,y)
    #plt.scatter(data[:,0], data[:,1])

    print len(x)

    #plt.minorticks_on()


    plt.xlim(-5, 5)
    plt.ylim(-5, 5)


    plt.show()


def bspline(cv, n=100, degree=3, periodic=False):
    """ Calculate n samples on a bspline

        cv :      Array ov control vertices
        n  :      Number of samples to return
        degree:   Curve degree
        periodic: True - Curve is closed
                  False - Curve is open
    """

    # If periodic, extend the point array by count+degree+1
    cv = np.asarray(cv)
    count = len(cv)

    if periodic:
        factor, fraction = divmod(count+degree+1, count)
        cv = np.concatenate((cv,) * factor + (cv[:fraction],))
        count = len(cv)
        degree = np.clip(degree,1,degree)

    # If opened, prevent degree from exceeding count-1
    else:
        degree = np.clip(degree,1,count-1)


    # Calculate knot vector
    kv = None
    if periodic:
        kv = np.arange(0-degree,count+degree+degree-1)
    else:
        kv = np.clip(np.arange(count+degree+1)-degree,0,count-degree)

    # Calculate query range
    u = np.linspace(periodic,(count-degree),n)


    # Calculate result
    return np.array(interpolate.splev(u, (kv,cv.T,degree))).T

if __name__ == "__main__":
    main()

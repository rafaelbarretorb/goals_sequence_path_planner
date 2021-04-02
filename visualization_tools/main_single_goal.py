#!/usr/bin/env python

# Test just one goal

from world import make_world

import numpy as np
import matplotlib.pyplot as plt
import pylab
import math

from helper_visual_functions import bspline, get_arrow_pose

from datetime import datetime

from goals_sequence_path_planner.rrt_star import RRT_Star
from goals_sequence_path_planner.maneuver_bubble import Maneuver

now = datetime.now()
PI = math.pi

# Scatter Plot
r = 2.0
scatter_area = PI*r*r

start_pose = [-2.0, -2.0, PI/4]
goal_pose = [-3.5, 3.5, PI/2]

goal_tolerance = 0.2

# FIGURE 1
grid_map, graph = make_world()

# make RRT* Path Planning
rrt_star = RRT_Star(start_point=start_pose, goal_point=goal_pose, grid=grid_map, min_num_nodes=1000,
                    max_num_nodes=5000, epsilon_min=0.0, epsilon_max=0.5, radius=0.5, goal_tolerance = 0.2,
                    obs_resolution=0.1, x_dim=10.0, y_dim=10.0, maneuvers=True)

path_x, path_y = rrt_star.path_planning()

later = datetime.now()
plt.plot(path_x[:], path_y[:])
plt.scatter(path_x, path_y)

########################################################

maneuver_radius = 0.5

# START Pose
start_maneuver = Maneuver(start_pose[0], start_pose[1], start_pose[2], 0.05, maneuver_radius, False)

x_start, y_start = start_maneuver.makeCenterCircle()
x_start_right, y_start_right = start_maneuver.makeRightCircle()
x_start_left, y_start_left = start_maneuver.makeLeftCircle()
x_start_arc, y_start_arc = start_maneuver.makeArc()

# GOAL Pose
goal_maneuver = Maneuver(goal_pose[0], goal_pose[1], goal_pose[2], goal_tolerance, maneuver_radius, True)

x_goal, y_goal = goal_maneuver.makeCenterCircle()
x_goal_right, y_goal_right = goal_maneuver.makeRightCircle()
x_goal_left, y_goal_left = goal_maneuver.makeLeftCircle()
x_goal_arc, y_goal_arc = goal_maneuver.makeArc()

# Plot Start
plt.plot(x_start,y_start)
plt.plot(x_start_right, y_start_right)
plt.plot(x_start_left, y_start_left)
plt.plot(x_start_arc, y_start_arc)

# Plot Goal
plt.plot(x_goal,y_goal)
plt.plot(x_goal_right, y_goal_right)
plt.plot(x_goal_left, y_goal_left)
plt.plot(x_goal_arc, y_goal_arc)

difference = (later - now).total_seconds()
########################################################
##### SPLINE #####
# data = list()
# for i in range(len(path_x)):
#   data.append([path_x[i], path_y[i]])

# data = np.array(data)
# p = bspline(data,n=100,degree=3)
# x,y = p.T
# plt.plot(x,y)

########################################################
# Plot Start Pose
start_endx, start_endy = get_arrow_pose(start_pose[0], start_pose[1], start_pose[2], arrow_length=0.25)
pylab.arrow(start_pose[0], start_pose[1], start_endx, start_endy, width=0.0075, color='red')

# Plot Goal
goal_endx, goal_endy = get_arrow_pose(goal_pose[0], goal_pose[1], goal_pose[2], arrow_length=0.25)
pylab.arrow(goal_pose[0], goal_pose[1], goal_endx, goal_endy, width=0.0075, color='black')



plt.show()
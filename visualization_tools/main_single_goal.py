#!/usr/bin/env python

from world import make_world
from rrt_star import RRT_Star
from orientation_filter import aim_to_next_position, get_arrows, get_arrow_pose


import numpy as np
import matplotlib.pyplot as plt
import pylab
import math
from maneuver_bubble import Maneuver

from spline_tests import bspline

from datetime import datetime

now = datetime.now()
PI = math.pi

# Scatter Plot
r = 2.0
scatter_area = PI*r*r

start_pose = [-4.0, 2.5, PI]
goal_pose = [4.5, 1.5, -PI/2]

# FIGURE 1
grid_map, graph = make_world()

# make RRT* Path Planning
rrt_star = RRT_Star(start_point=start_pose, goal_point=goal_pose, grid=grid_map,
                    max_num_nodes=5000, epsilon_min=0.0, epsilon_max=0.5, obs_resolution=0.1, maneuvers=True)

path_x, path_y = rrt_star.path_planning()

later = datetime.now()
plt.plot(path_x[:], path_y[:])
#plt.scatter(path_x, path_y)

########################################################
# goal_tolerance = 0.2
# maneuver_radius = 0.5

# # START Pose
# start_maneuver = Maneuver(start_pose[0], start_pose[1], start_pose[2], 0.05, maneuver_radius, False)

# x_start, y_start = start_maneuver.makeCenterCircle()
# x_start_right, y_start_right = start_maneuver.makeRightCircle()
# x_start_left, y_start_left = start_maneuver.makeLeftCircle()
# x_start_arc, y_start_arc = start_maneuver.makeArc()

# # GOAL Pose
# goal_maneuver = Maneuver(goal_pose[0], goal_pose[1], goal_pose[2], goal_tolerance, maneuver_radius, True)

# x_goal, y_goal = goal_maneuver.makeCenterCircle()
# x_goal_right, y_goal_right = goal_maneuver.makeRightCircle()
# x_goal_left, y_goal_left = goal_maneuver.makeLeftCircle()
# x_goal_arc, y_goal_arc = goal_maneuver.makeArc()

# # Plot Start
# plt.plot(x_start,y_start)
# plt.plot(x_start_right, y_start_right)
# plt.plot(x_start_left, y_start_left)
# plt.plot(x_start_arc, y_start_arc)

# # Plot Goal
# plt.plot(x_goal,y_goal)
# plt.plot(x_goal_right, y_goal_right)
# plt.plot(x_goal_left, y_goal_left)
# plt.plot(x_goal_arc, y_goal_arc)

difference = (later - now).total_seconds()
print difference
########################################################
##### SPLINE #####
data = list()
for i in range(len(path_x)):
  data.append([path_x[i], path_y[i]])

data = np.array(data)
p = bspline(data,n=100,degree=3)
x,y = p.T
plt.plot(x,y)

########################################################

# Plot Goal
goal_endx, goal_endy = get_arrow_pose(goal_pose[0], goal_pose[1], goal_pose[2], arrow_length=0.5)
pylab.arrow(goal_pose[0], goal_pose[1], goal_endx, goal_endy, width=0.01, color='black')

# Plot Goal
start_endx, start_endy = get_arrow_pose(start_pose[0], start_pose[1], start_pose[2], arrow_length=0.5)
pylab.arrow(start_pose[0], start_pose[1], start_endx, start_endy, width=0.01, color='red')

plt.show()
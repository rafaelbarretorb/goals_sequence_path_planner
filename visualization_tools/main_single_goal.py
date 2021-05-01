#!/usr/bin/env python

# Test just one goal

from world import make_world

import numpy as np
import matplotlib.pyplot as plt
import pylab
import math

from helper_visual_functions import bspline, get_arrow_pose

from datetime import datetime

from goals_sequence_path_planner.rrt_star_smart_dual_tree import RRTStarSmartDualTree
from goals_sequence_path_planner.virtual_obstacle import VirtualObstacle

now = datetime.now()
PI = math.pi

# Scatter Plot
r = 2.0
scatter_area = PI*r*r

start_pose = [-3.5, 3.5, PI]
goal_pose = [3.5, 0.5, -PI/2]

# start_pose = [-3.5, 3.5, 0.0]
# goal_pose = [2.0, 2.5, -PI/2]

goal_tolerance = 0.2

# FIGURE 1
grid_map, graph = make_world()

planner = RRTStarSmartDualTree(start_pose=start_pose,
				               goal_pose=goal_pose,
				               grid=grid_map,
				               min_num_nodes=1000,
                               max_num_nodes=2000,
                               goal_tolerance=goal_tolerance,
				               epsilon=0.5,
				               optimization_radius=1.0,
                               obs_resolution=0.05,
				               biasing_radius=1.0,
				               biasing_ratio=20,
				               x_dim=10.0,
				               y_dim=10.0,
				               virtual_obstacles=True)

path = planner.planning()

path_x = list()
path_y = list()
for p in path:
	path_x.append(p[0])
	path_y.append(p[1])

later = datetime.now()
plt.plot(path_x, path_y)
# plt.scatter(path_x, path_y)

########################################################

maneuver_radius = 0.5
goal_tolerance = 0.05
obs_color = 'gray'

# START Pose
start_virtual_obs = VirtualObstacle(start_pose, goal_tolerance, maneuver_radius, False)

x_start, y_start = start_virtual_obs.make_center_circle()
x_start_right, y_start_right = start_virtual_obs.make_right_circle()
x_start_left, y_start_left = start_virtual_obs.make_left_circle()
x_start_arc, y_start_arc = start_virtual_obs.make_arc()

# GOAL Pose
goal_virtual_obs = VirtualObstacle(goal_pose, goal_tolerance, maneuver_radius, True)

x_goal, y_goal = goal_virtual_obs.make_center_circle()
x_goal_right, y_goal_right = goal_virtual_obs.make_right_circle()
x_goal_left, y_goal_left = goal_virtual_obs.make_left_circle()
x_goal_arc, y_goal_arc = goal_virtual_obs.make_arc()

# Plot Start Virtual Obstacles
plt.plot(x_start, y_start, color=obs_color)
plt.plot(x_start_right, y_start_right, color=obs_color)
plt.plot(x_start_left, y_start_left, color=obs_color)
plt.plot(x_start_arc, y_start_arc, color=obs_color)

# Plot Goal
plt.plot(x_goal,y_goal, color=obs_color)
plt.plot(x_goal_right, y_goal_right, color=obs_color)
plt.plot(x_goal_left, y_goal_left, color=obs_color)
plt.plot(x_goal_arc, y_goal_arc, color=obs_color)

# difference = (later - now).total_seconds()
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
start_endx, start_endy = get_arrow_pose(start_pose, arrow_length=0.3)
pylab.arrow(start_pose[0], start_pose[1], start_endx, start_endy, width=0.05, color='red')

# Plot Goal
goal_endx, goal_endy = get_arrow_pose(goal_pose, arrow_length=0.3)
pylab.arrow(goal_pose[0], goal_pose[1], goal_endx, goal_endy, width=0.05, color='green')



plt.show()
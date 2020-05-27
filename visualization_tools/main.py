#!/usr/bin/env python

from world import make_world
from rrt_star import RRT_Star
from orientation_filter import aim_to_next_position, get_arrows, get_arrow_pose


import numpy as np
import matplotlib.pyplot as plt
import pylab
import math

from global_path_planner import GlobalPathPlanner

from datetime import datetime

time = datetime.now()
PI = math.pi

# Scatter Plot
r = 2.0
scatter_area = PI*r*r

##### TEST 1 ######
start_pose = [-3.0, -4.0, PI]
goals_list = [[-4.0, 2.0], [2.5, 1.5], [3.0, -4.5]]
# goals_list = [[3.5, -4.5], [2.5, 1.5], [-4.0, 2.5]]
goals_list2 = goals_list[:]
#goals_list2 = [[3.5, -4.5], [2.5, 1.5], [-4.0, 2.5]]
final_pose = [-3.0, -4.0, PI/2]

##### TEST 2 ######
# start_pose = [-3.0, -4.0, 0]
# goals_list = [[2.5, 1.5], [3.0, -4.5]]
# goals_list2 = [[2.5, 1.5], [3.0, -4.5]]
# final_pose = [-3.0, -4.0, PI/2]

##### TEST 3 ######
# start_pose = [-3.0, -4.0, PI/2]
# goals_list = [[-4.0, 2.5], [3.5, 3.5], [3.0, -3.0]]
# goals_list2 = [[-4.0, 2.5], [3.5, 3.5], [3.0, -3.0]]
# final_pose = [-1.0, -4.0, PI/2]

##############################
########## FIGURE 1 ##########
##############################
grid_map, graph = make_world()

# TIME
time1 = datetime.now()
print "Time 1: " + str((time1 - time).total_seconds())

planner = GlobalPathPlanner(start_pose=start_pose, global_map=grid_map, final_pose=final_pose)

# TIME
time2 = datetime.now()
print "Time 2: " + str((time2 - time).total_seconds())

paths = planner.usual_paths(goals_list)

# TIME
time3 = datetime.now()
print "Time 3: " + str((time3 - time).total_seconds())


# get each goal orientation

# I

for i in range(len(goals_list)):
    yaw, correct_yaw = planner.get_goal_orientation(paths[i], paths[i+1])
    goals_list[i].append(yaw)
    goals_list2[i].append(correct_yaw)

goals_list.append(final_pose)
goals_list2.append(final_pose)

colors = ["green", "blue", "red", "orange"]

for i in range(len(paths)):
  x = list()
  y = list()
  for j in range(len(paths[i])):
    x.append(paths[i][j][0])
    y.append(paths[i][j][1])
  plt.plot(x,y, color=colors[i])

  # F
  endx, endy = get_arrow_pose(paths[i][-1][0], paths[i][-1][1], goals_list[i][2], arrow_length=0.3)
  pylab.arrow(paths[i][-1][0], paths[i][-1][1], endx, endy, width=0.0075, color='black')

  endx, endy = get_arrow_pose(paths[i][-1][0], paths[i][-1][1], goals_list2[i][2], arrow_length=0.3)
  pylab.arrow(paths[i][-1][0], paths[i][-1][1], endx, endy, width=0.0075, color='green')

start_endx, start_endy = get_arrow_pose(start_pose[0], start_pose[1], start_pose[2], arrow_length=0.3)
pylab.arrow(start_pose[0], start_pose[1], start_endx, start_endy, width=0.0075, color='red')



points = goals_list[:]
points.insert(0, start_pose)
##############################
########## FIGURE 2 ##########
##############################
grid_map, graph = make_world()

# TIME
time4 = datetime.now()
print "Time 4: " + str((time4 - time).total_seconds())

opt_paths = planner.optimized_paths(points)


# TIME
time5 = datetime.now()
print "Time 5: " + str((time4 - time).total_seconds())

for i in range(len(opt_paths)):
  # if the path size is zero than use the usual path
  if len(opt_paths[i]) == 0:
    print i
    opt_paths[i] = paths[i][:]
    if i > 0:
      opt_paths[i][0] = opt_paths[i-1][-1]

  x = list()
  y = list()
  for j in range(len(opt_paths[i])):
    x.append(opt_paths[i][j][0])
    y.append(opt_paths[i][j][1])
  plt.plot(x,y, color=colors[i])

  endx, endy = get_arrow_pose(opt_paths[i][-1][0], opt_paths[i][-1][1], goals_list[i][2], arrow_length=0.3)
  pylab.arrow(opt_paths[i][-1][0], opt_paths[i][-1][1], endx, endy, width=0.0075, color='green')

start_endx, start_endy = get_arrow_pose(start_pose[0], start_pose[1], start_pose[2], arrow_length=0.3)
pylab.arrow(start_pose[0], start_pose[1], start_endx, start_endy, width=0.0075, color='red')


##############################
########## FIGURE 3 ##########
##############################
grid_map, graph = make_world()
smoother_paths = planner.make_paths_smoother(opt_paths)

for i in range(len(smoother_paths)):
  x, y = smoother_paths[i].T
  plt.plot(x,y, color=colors[i])

plt.show()
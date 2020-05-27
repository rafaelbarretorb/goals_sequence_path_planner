#!/usr/bin/env python

from world import make_world

import matplotlib.pyplot as plt
import pylab
import math

from datetime import datetime


from global_path_planner import GlobalPathPlanner

PI = math.pi

# Scatter Plot
r = 2.0
scatter_area = PI*r*r


##### TEST 3 ######
start_pose = [-3.0, -4.0, PI/2]
goals_list = [[-4.0, 2.], [2.5, 1.5]]# [3.0, -3.0]]
final_pose = [-1.0, -4.0, PI/2]

colors = ["green", "blue", "red", "orange"]


grid_map, graph = make_world()


now = datetime.now()
planner = GlobalPathPlanner(start_pose=start_pose, global_map=grid_map, final_pose=final_pose)
smoother_paths = planner.global_path_planner(goals_list)
later = datetime.now()

difference = (later - now).total_seconds()
print "Time difference: " + str(difference)


print "Paths List Length: " + str(len(smoother_paths))
for i in range(len(smoother_paths)):
  x, y = smoother_paths[i].T
  plt.plot(x,y, color=colors[i])

plt.show()
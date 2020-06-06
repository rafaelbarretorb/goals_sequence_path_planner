#!/usr/bin/env python

# NEW CONCEPT

import math
import pylab
import matplotlib.pyplot as plt
import copy

from goals_sequence_path_planner.global_path_planner import GlobalPathPlanner
from world import make_world

from helper_visual_functions import bspline, get_arrow_pose
# from orientation_filter import aim_to_next_position, get_arrows, get_arrow_pose

PI = math.pi

G1 = [-3.0, 3.0, PI, True]
G2 = [-0.5, 3.0, None, True]
G3 = [3.0, 0.5, PI/2, True]
G4 = [3.0, -3.5, PI/2, True]

# TEST 1
goals_list = [G1, G3, G4]
start_pose = [-3.5, -4.0, PI/2, False]
final_pose = [-2.5, -4.0, -PI/2, False]

# # TEST 2
# goals_list = [G1, G3, G4, G3]
# start_pose = [-3.5, -2.0, PI/2, False]
# final_pose = [-2.5, -2.0, -PI/2, False]

# # TEST 3
# goals_list = [G3, G4, G3]
# start_pose = [-3.5, -2.0, PI/2, False]
# final_pose = [-2.5, -2.0, -PI/2, False]

# goals_list = [G1, G2, G3, G4]
# start_pose = [-3.5, -4.0, PI/2, False]
# final_pose = [-2.5, -4.0, -PI/2, False]

# goals_list = [G1, G2, G3]
# start_pose = [-3.5, -4.0, PI/2, False]
# final_pose = [-2.5, -4.0, -PI/2, False]

# goals_list = [G1, G3, G2, G4]
# start_pose = [-3.5, -4.0, PI/2, False]
# final_pose = [-2.5, -4.0, -PI/2, False]

# TEST 4
# goals_list = [G1]
# start_pose = [-3.5, -4.0, PI/2, False]
# final_pose = [-2.5, -4.0, -PI/2, False]

grid_map, graph = make_world()
planner = GlobalPathPlanner(start_pose=start_pose, global_map=grid_map, final_pose=final_pose)

paths = planner.usual_paths(goals_list)

dod_angles = list()
doa_angles = list()

goals_angles = list()

for i in range(len(goals_list)):
	dod, doa = planner.get_goal_orientation(paths[i], paths[i+1])
	dod_angles.append(dod)
	doa_angles.append(doa)
	goals_angles.append(goals_list[i][2])

# print [math.degrees(goals_angles[0]), math.degrees(goals_angles[1]), math.degrees(goals_angles[2])]

goals_angles = planner.get_best_angle(dod_angles, goals_angles)

# print [math.degrees(goals_angles[0]), math.degrees(goals_angles[1]), math.degrees(goals_angles[2])]

# DOA
# for i in range(len(doa_angles)):
	

for i in range(len(goals_angles)):
    goals_list[i][2] = goals_angles[i]

colors = ["green", "blue", "red", "orange", "black"]

for i in range(len(paths)):
	x = list()
	y = list()
	for j in range(len(paths[i])):
		x.append(paths[i][j][0])
		y.append(paths[i][j][1])
	plt.plot(x,y, color=colors[i])
	#plt.scatter(x,y, color=colors[i])

	# DOD
	# endx, endy = get_arrow_pose(paths[i][-1][0], paths[i][-1][1], goals_list[i][2], arrow_length=0.3)
	# pylab.arrow(paths[i][-1][0], paths[i][-1][1], endx, endy, width=0.0075, color='black')

#   # Arrival Angles
#   endx, endy = get_arrow_pose(paths[i][-1][0], paths[i][-1][1], goals_list3[i][2], arrow_length=0.3)
#   pylab.arrow(paths[i][-1][0], paths[i][-1][1], endx, endy, width=0.0075, color='green')

#   # endx, endy = get_arrow_pose(paths[i][-1][0], paths[i][-1][1], goals_list2[i][2], arrow_length=0.3)
#   # pylab.arrow(paths[i][-1][0], paths[i][-1][1], endx, endy, width=0.0075, color='green')

start_endx, start_endy = get_arrow_pose(start_pose[0], start_pose[1], start_pose[2], arrow_length=0.3)
pylab.arrow(start_pose[0], start_pose[1], start_endx, start_endy, width=0.0075, color='red')


points = copy.deepcopy(goals_list)
points.insert(0, start_pose)
points.append(final_pose)

# points2 = copy.deepcopy(goals_list2)
# points2.insert(0, start_pose)


# points3 = copy.deepcopy(goals_list3)
# points3.insert(0, start_pose)
##############################
########## FIGURE 2 ##########
##############################
grid_map, graph = make_world()

opt_paths = planner.optimized_paths(points)

for i in range(len(opt_paths)):
  # if the path size is zero than use the usual path
  if len(opt_paths[i]) == 0:
    opt_paths[i] = paths[i][:]
    if i > 0:
      opt_paths[i][0] = opt_paths[i-1][-1]

  x = list()
  y = list()
  for j in range(len(opt_paths[i])):
    x.append(opt_paths[i][j][0])
    y.append(opt_paths[i][j][1])
  plt.plot(x,y, color=colors[i])




grid_map, graph = make_world()
smoother_paths = planner.make_paths_smoother(opt_paths)

for i in range(len(smoother_paths)):
	x, y = smoother_paths[i].T
	plt.plot(x,y, color=colors[i])

start_endx, start_endy = get_arrow_pose(start_pose[0], start_pose[1], start_pose[2], arrow_length=0.3)
pylab.arrow(start_pose[0], start_pose[1], start_endx, start_endy, width=0.0075, color='red')

endx, endy = get_arrow_pose(x[-1], y[-1], final_pose[2], arrow_length=0.3)
pylab.arrow(x[-1], y[-1], endx, endy, width=0.0075, color='green')

plt.show()

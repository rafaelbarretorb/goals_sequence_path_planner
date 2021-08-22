#!/usr/bin/env python

# NEW CONCEPT
import numpy as np
import math
import copy

from goals_sequence_path_planner.sequence_of_goals_planner import SequenceOfGoalsPlanner
from helper_visual_functions import get_arrow_pose, make_spline_curve
from plot_functions import *

PI = math.pi


def main():
	min_num_nodes = 1000
	max_num_nodes = 2000
	goal_tolerance = 0.2
	epsilon = 0.5
	optimization_radius = 1.0
	obs_resolution = 0.05
	biasing_radius = 1.0
	biasing_ratio = 20
	x_dim = 10.0
	y_dim = 10.0



	# TODO put separate file these variables
	start_pose = [-3.5, -3.0, PI/4, True]
	final_pose = [-3.0, -3.5, PI/4, True]
	G1 = [-3.5, 3.75, PI, True]
	G2 = [-0.5, 3.75, None, True]
	G3 = [3.75, 0.5, PI/2, True]
	G4 = [3.75, -3.5, PI/2, True]

	# -----------------
	# ----- TESTS -----
	# -----------------

	# goals_list = [G1]
	# goals_list = [G4, G1]
	goals_list = [G1, G3, G4]
	# goals_list = [G1, G2, G3, G4]
	# goals_list = [G2]

	# --------------------
	# ----- FIGURE 1 -----
	# --------------------

	grid_map = make_new_world()

	planner = SequenceOfGoalsPlanner(start_pose=start_pose,
									 final_pose=final_pose,
									 goals_list=goals_list,
									 global_map=grid_map,
									 min_num_nodes=min_num_nodes,
									 max_num_nodes=max_num_nodes,
									 goal_tolerance=goal_tolerance,
									 epsilon=epsilon,
									 optimization_radius=optimization_radius,
									 obs_resolution=obs_resolution,
									 biasing_radius=biasing_radius,
									 biasing_ratio=biasing_ratio,
									 x_dim=x_dim,
									 y_dim=y_dim)

	paths = planner.get_usual_paths()
	dod_angles = planner.get_dod_angles()
	doa_angles = planner.get_doa_angles()

	#
	# paths = [[(-1, -2), (0, 0)], [(0, 0), (1, -2)]]

	# Plot DOA and DOD
	plot_paths(paths)
	# plot_dod_doa(paths, dod_angles, doa_angles)

	# Start and Final poses
	plot_pose(start_pose, 'purple')
	plot_pose(final_pose, 'orange')

	# Plot Goals List
	plot_goals(goals_list, 'black')

	# plot_goals_position(paths)

	# Plot Text Workbenchs
	plot_text()

	# -------------------------------------------
	# ----- Master Dissertation Figure 3.18 -----
	# -------------------------------------------
	plot_dod_doa(paths, dod_angles, doa_angles)

	# -------------------------------------------

	# --------------------
	# ----- FIGURE 2 -----
	# --------------------

	grid_map = make_new_world()

	opt_paths = planner.get_optimized_paths()

	# opt_paths = [[(2, 2), (3, 3)], [(3, 3), (4, 2)]]

	# print "Optimized Paths: " + str(opt_paths)
	plot_paths(opt_paths)

	# Start and Final poses
	plot_pose(start_pose, 'purple')
	plot_pose(final_pose, 'orange')

	# Plot Goals List
	plot_goals(goals_list, 'black')

	# Plot Text Workbenchs
	plot_text()

    # --------------------
    # ----- FIGURE 3 -----
    # --------------------

	make_new_world()

	# print "Length opt_paths: " + str(len(opt_paths))
	# for i in range(len(opt_paths)):
	# 	# if the path size is zero than use the usual path
	# 	if len(opt_paths[i]) == 0:
	# 		opt_paths[i] = paths[i][:]
	# 		if i > 0:
	# 			opt_paths[i][0] = opt_paths[i-1][-1]

	# 	print "Length opt_paths[" + str(i) + "]: " + str(len(opt_paths[i]))
	# 	data = list()
	# 	for j in range(len(opt_paths[i])):
	# 		data.append([opt_paths[i][j][0], opt_paths[i][j][1]])

	i = 0
	for path in opt_paths:
		spline_curve, cv = make_spline_curve(path, 0.5, 0.35)
		x, y = spline_curve.T
		plt.plot(x, y, color=colors[i])
		i = i + 1

	# Start and Final poses
	plot_pose(start_pose, 'purple')
	plot_pose(final_pose, 'orange')

	# Plot Goals List
	plot_goals(goals_list, 'black')

	# Plot Text Workbenchs
	plot_text()


if __name__ == "__main__":
    main()
    plt.show()

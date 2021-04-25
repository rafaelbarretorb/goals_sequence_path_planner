#!/usr/bin/env python

# NEW CONCEPT
import numpy as np
import math
import copy

from goals_sequence_path_planner.sequence_of_goals_planner import SequenceOfGoalsPlanner
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
	start_pose = [-3.0, -3.0, PI/4, True]
	# start_pose = [-3.5, -4.0, PI/2, True]
	# start_pose = G1
	# final_pose = [-2.5, -4.0, -PI/2, True]
	final_pose = [-2.0, -3.0, PI/4, True]
	G1 = [-3.5, 3.75, PI, True]
	G2 = [3.5, 2.0, PI/2, True]
	# G2 = [-0.5, 3.5, None, True]
	G3 = [3.75, 0.5, PI/2, True]
	G4 = [3.75, -3.5, PI/2, True]

	# -----------------
	# ----- TESTS -----
	# -----------------

	# TEST 1
	goals_list = [G1]

	# # TEST 2
	# goals_list = [G4, G1]

	# TEST 3
	goals_list = [G1, G3, G4]

	# TEST 4
	# goals_list = [G1, G2, G3, G4]
	# goals_list = [G1, G2]


	# goals_list = [G1, G2, G3]

	# goals_list = [G1, G2, G3, G4]


	# TEST 4
	# goals_list = [G1]
	# start_pose = [-3.5, -4.0, PI/2, False]
	# final_pose = [-2.5, -4.0, -PI/2, False]


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

	# Plot
	# print "Usual Paths: " + str(paths)
	plot_paths(paths)
	plot_dod_doa(paths, dod_angles, doa_angles)

	# Start Pose
	plot_pose(start_pose[0], start_pose[1], start_pose[2], 'purple')

	# Final Pose
	plot_pose(paths[-1][-1][0], paths[-1][-1][1], final_pose[2], 'orange')

	# plot_goals_position(paths)

	# --------------------
	# ----- FIGURE 2 -----
	# --------------------

	grid_map = make_new_world()

	opt_paths = planner.get_optimized_paths()

	# opt_paths = [[(2, 2), (3, 3)], [(3, 3), (4, 2)]]

	# print "Optimized Paths: " + str(opt_paths)
	plot_paths(opt_paths)

	# Start Pose
	plot_pose(start_pose[0], start_pose[1], start_pose[2], 'purple')

	# Final Pose
	plot_pose(opt_paths[-1][-1][0], opt_paths[-1][-1][1], final_pose[2], 'orange')

    # --------------------
    # ----- FIGURE 3 -----
    # --------------------

    # make_new_world()

    # print "Length opt_paths: " + str(len(opt_paths))
    # for i in range(len(opt_paths)):
    # # if the path size is zero than use the usual path
    # if len(opt_paths[i]) == 0:
    #     opt_paths[i] = paths[i][:]
    #     if i > 0:
    #     opt_paths[i][0] = opt_paths[i-1][-1]

    # print "Length opt_paths[" + str(i) + "]: " + str(len(opt_paths[i]))
    # data = list()
    # for j in range(len(opt_paths[i])):
    #     data.append([opt_paths[i][j][0], opt_paths[i][j][1]])

    # data = np.array(data)
    # p = bspline(data, n=100, degree=3)
    # x, y = p.T
    # plt.plot(x, y, color=colors[i])

# def plot_paths(paths, scatter=False):
#     for i in range(len(paths)):
#         x = list()
#         y = list()
#         for j in range(len(paths[i])):
#             x.append(paths[i][j][0])
#             y.append(paths[i][j][1])
        
#         # Plot
#         plt.plot(x,y, color=colors[i])

#         # Scatter
#         if scatter:
#             plt.scatter(x, y, color=colors[i])

# def plot_dod_doa(paths, dod_angles, doa_angles):
#     for i in range(len(dod_angles)):
#         x = paths[i][-1][0]
#         y = paths[i][-1][1]

#         # DOA
#         doa = doa_angles[i]
#         plot_pose(x, y, doa, 'red')

#         # DOD
#         dod = dod_angles[i]
#         plot_pose(x, y, dod, 'black')

# def make_new_world():
#     grid_map, graph = make_world()
#     return grid_map

# def plot_pose(x, y, yaw, color):
#     endx, endy = get_arrow_pose(x, y, yaw, arrow_length=0.3)
#     pylab.arrow(x, y, endx, endy, width=arrow_width, color=color)

# def plot_goals_position(paths):
#     x = list()
#     y = list()
#     for i in range(len(paths) - 1):
#         x.append(paths[i][-1][0])
#         y.append(paths[i][-1][1])

#     plt.scatter(x, y, color='black')

# def plot_virtual_obs(goals):
#     pass

if __name__ == "__main__":
    main()
    plt.show()

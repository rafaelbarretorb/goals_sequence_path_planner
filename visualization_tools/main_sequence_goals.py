#!/usr/bin/env python

# NEW CONCEPT
import numpy as np
import math
import pylab
import matplotlib.pyplot as plt
import copy

from goals_sequence_path_planner.global_path_planner import SequenceOfGoalsPlanner
from world import make_world

from helper_visual_functions import bspline, get_arrow_pose

PI = math.pi

colors = ["green", "blue", "red", "orange", "black"]

# Arrow Pose
arrow_width = 0.05
arrow_length = 0.3

def main():

    # TODO put separate file these variables
    start_pose = [-3.0, -3.0, PI/4, True]
    # start_pose = [-3.5, -4.0, PI/2, True]
    # start_pose = G1
    # final_pose = [-2.5, -4.0, -PI/2, True]
    final_pose = [-2.0, -3.0, PI/4, True]
    G1 = [-3.5, 3.75, PI, True]
    G2 = [3.5, 2.0, PI/2, True]
    # G2 = [-0.5, 3.5, None, True]
    G3 = [3.5, 0.5, PI/2, True]
    G4 = [3.5, -3.5, PI/2, True]

    # -----------------
    # ----- TESTS -----
    # -----------------

    # TEST 1
    goals_list = [G1]

    # # TEST 2
    # goals_list = [G4, G1]

    # TEST 3
    # goals_list = [G1, G3, G4]

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
                                x_dim=10.0,
                                y_dim=10.0)

    paths = planner.get_usual_paths()
    dod_angles = planner.get_dod_angles()
    doa_angles = planner.get_doa_angles()

    # Plot
    plot_paths(paths, scatter=True)
    plot_dod_doa(paths, dod_angles, doa_angles)
    
    # Start Pose
    plot_pose(start_pose[0], start_pose[1], start_pose[2], 'purple')

    # Final Pose
    plot_pose(paths[-1][-1][0], paths[-1][-1][1], final_pose[2], 'orange')

    # --------------------
    # ----- FIGURE 2 -----
    # --------------------

    make_new_world()

    opt_paths = planner.get_optimized_paths()
    plot_paths(opt_paths)

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

def plot_paths(paths, scatter=False):
    for i in range(len(paths)):
        x = list()
        y = list()
        for j in range(len(paths[i])):
            x.append(paths[i][j][0])
            y.append(paths[i][j][1])
        
        # Plot
        plt.plot(x,y, color=colors[i])

        # Scatter
        if scatter:
            plt.scatter(x, y, color=colors[i])

def plot_dod_doa(paths, dod_angles, doa_angles):
    for i in range(len(dod_angles)):
        x = paths[i][-1][0]
        y = paths[i][-1][1]

        # DOA
        doa = doa_angles[i]
        plot_pose(x, y, doa, 'red')

        # DOD
        dod = dod_angles[i]
        plot_pose(x, y, dod, 'brown')

def make_new_world():
    grid_map, graph = make_world()
    return grid_map

def plot_pose(x, y, yaw, color):
    endx, endy = get_arrow_pose(x, y, yaw, arrow_length=0.3)
    pylab.arrow(x, y, endx, endy, width=arrow_width, color=color)

if __name__ == "__main__":
    main()
    plt.show()

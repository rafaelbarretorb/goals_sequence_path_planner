#!/usr/bin/env python

import pylab
import matplotlib.pyplot as plt

from world import make_world
from helper_visual_functions import bspline, get_arrow_pose

colors = ["green", "blue", "red", "orange", "black"]

# Arrow Pose
arrow_width = 0.05
arrow_length = 0.3

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
        plot_pose(x, y, dod, 'black')

def make_new_world():
    grid_map, graph = make_world()
    return grid_map

def plot_pose(x, y, yaw, color):
    endx, endy = get_arrow_pose(x, y, yaw, arrow_length=0.3)
    pylab.arrow(x, y, endx, endy, width=arrow_width, color=color)

def plot_goals_position(paths):
    x = list()
    y = list()
    for i in range(len(paths) - 1):
        x.append(paths[i][-1][0])
        y.append(paths[i][-1][1])

    plt.scatter(x, y, color='black')

def plot_virtual_obs(goals):
    pass
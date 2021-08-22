#!/usr/bin/env python

import pylab
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 18})

from world import make_world
from helper_visual_functions import bspline, get_arrow_pose

colors = ["green", "blue", "red", "orange", "black"]

# Arrow Pose
arrow_width = 0.05
arrow_length = 0.3

def plot_single_path(path, color, scatter=False, alpha=1.0, linewidth=1):
	""" ."""
	x, y = zip(*path)
	plt.plot(x,y, color=color, alpha=alpha, linewidth=linewidth)

	if scatter:
		plt.scatter(x, y, color=color)

def plot_paths(paths, scatter=False):
	print "Paths Length: " + str(len(paths))
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
		plot_pose([x, y, doa], 'red')

		# DOD
		dod = dod_angles[i]
		plot_pose([x, y, dod], 'black')

def plot_goals(goals_list, color):
	for goal in goals_list:
		plt.scatter(goal[0], goal[1], color=color)

def make_new_world():
	grid_map, graph = make_world()
	return grid_map

def plot_pose(pose, color):
	x = pose[0]
	y = pose[1]
	yaw = pose[2]
	endx, endy = get_arrow_pose(yaw, arrow_length=0.3)
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

def plot_text():
	# Workbench 1
	s = '1'
	x = -3.7
	y = 4.3
	plt.text(x, y, s, color='yellow', fontweight='bold')

	# Workbench 2
	s = '2'
	x = -0.7
	y = 4.3
	plt.text(x, y, s, color='yellow', fontweight='bold')

	# Workbench 3
	s = '3'
	x = 4.3
	y = 0.35
	plt.text(x, y, s, color='yellow', fontweight='bold')

	# Workbench 4
	s = '4'
	x = 4.3
	y = -3.65
	plt.text(x, y, s, color='yellow', fontweight='bold')

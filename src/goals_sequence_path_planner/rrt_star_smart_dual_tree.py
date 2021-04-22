#!/usr/bin/env python

import pygame
import sys

from constants import GREEN, RED, BLACK, WHITE, GRAY
from tree import Tree

from obstacles import Obstacles


class RRTStarSmartDualTree:
	""" Class for RRT*-Smart Dual Tree Path Planning. """
	def __init__(self,
	             start_point,
				 goal_point,
				 grid,
				 min_num_nodes,
                 max_num_nodes,
                 goal_tolerance,
				 epsilon,
				 optimization_radius,
                 obs_resolution,
				 biasing_ratio,
				 x_dimension,
				 y_dimension,
				 maneuvers=False):

		self.obs_resolution = obs_resolution

		self.start_point = start_point
		self.goal_point = goal_point

		self.max_num_nodes = max_num_nodes
		self.min_num_nodes = min_num_nodes
		self.epsilon = epsilon

		self.goal_tolerance = goal_tolerance

		self.start_tree = Tree(True,
								start_point,
								goal_tolerance=20,
								epsilon_min=epsilon_min,
								epsilon_max=epsilon_max,
								max_num_nodes=5000,
								screen=self.screen,
								obstacles=self.obstacles,
								obs_resolution=self.obs_resolution,
								biasing_radius=20.0)

		self.goal_tree = Tree(False,
								goal_point,
								goal_tolerance=20,
								epsilon_min=epsilon_min,
								epsilon_max=epsilon_max,
								max_num_nodes=5000,
								screen=self.screen,
								obstacles=self.obstacles,
								obs_resolution=self.obs_resolution,
								biasing_radius=20.0)

		self.tree = None

		self.goal_found = False

		self.n = None  # iteration where initial path found
		self.it = 0
		self.biasing_ratio = biasing_ratio

	def valid_start_and_goal(self):
		""" ."""
        if self.start_tree.collision(self.start_point):
            sys.exit("ERROR!!!!. Start Position is not allowed.")

        if self.goal_tree.collision(self.goal_point):
            sys.exit("ERROR!!!!. Goal Position is not allowed.")

	def planning(self):
		""" ."""
		j = 1
		first_path_computed = False
		while self.keep_searching():
			if self.n != None and self.it == (self.n + j*self.biasing_ratio):
				self.tree.grow_tree(random_sample=False)
				j = j + 1
				
			else:
				# Start Tree's turn
				self.run_tree(self.start_tree, self.goal_tree)

				# Goal Tree's turn
				self.run_tree(self.goal_tree, self.start_tree)

				if self.goal_found:
					path = self.tree.path_optimization()

			# Iteration
			self.it = self.it + 1

		return path

	def run_tree(self, tree_obj, other_tree_obj):
		""" ."""
		if tree_obj.is_tree_blocked():
			return
		else:
			# Tree grows
			tree_obj.grow_tree()

			# Tree new node
			new_node = tree_obj.get_new_node()

			if not self.goal_found:
				if other_tree_obj.attempt_connect(new_node):
					self.goal_found = True

					# Set n
					it = self.it
					self.n = it

					# Block Other Tree
					other_tree_obj.block_tree()

					# # TODO review remove
					self.tree = tree_obj

					n_nearest_ext = other_tree_obj.get_n_nearest_external()
					
					# Get nodes path from OTHER Tree
					external_nodes = other_tree_obj.get_external_nodes(n_nearest_ext)
					
					#  Add nodes path to Tree
					tree_obj.add_nodes_to_tree(external_nodes, new_node)

					path = self.tree.compute_path()

	def keep_searching(self):
		""" ."""
		start_size = self.start_tree.get_nodes_length()
		goal_size = self.goal_tree.get_nodes_length()
		if start_size > self.max_num_nodes or goal_size > self.max_num_nodes:
			return False
		else:
			return True

#!/usr/bin/env python

import random
from helper_functions import * # dist
from node import Node
import sys
from maneuver_bubble import Maneuver
import numpy as np


class Tree:
	def __init__(self,
	             is_start_tree,
				 start_point,
				 start_point_other_tree,
				 grid,
				 goal_tolerance,
				 epsilon,
				 max_num_nodes,
				 obs_resolution,
				 optimization_radius,
				 x_dim,
				 y_dim,
				 biasing_radius=None,
				 virtual_obstacles=False):
		self.is_start_tree = is_start_tree
		self.start_point = start_point

		# TODO Improve This
		self.goal_point = start_point_other_tree
		self.grid = grid
		self.goal_tolerance = goal_tolerance
		self.epsilon = epsilon
		self.max_num_nodes = max_num_nodes
		self.obs_resolution = obs_resolution
		self.optimization_radius = optimization_radius
		self.x_dim = x_dim	
		self.y_dim = y_dim
		self.biasing_radius = biasing_radius
		self.virtual_obstacles = virtual_obstacles

		# RRT*-Smart Beacons
		self.beacons = list()

		# Tree nodes
		self.nodes = list()

		# Add the first node
		self.nodes.append(Node(self.start_point, None))

		self.new_node = None
		self.n_nearest_ext = None
		self.goal = None
		self.tree_blocked = False
		self.path_old = list()


		virtual_obs_radius = 0.5
		virtual_obs_tolerance = 0.05

		# TODO Improve this shit
		if not self.is_start_tree:
			start_virtual_obs = True
			goal_virtual_obs = False
		else:
			start_virtual_obs = False
			goal_virtual_obs = True

		if self.virtual_obstacles:
			self.start_maneuver = Maneuver(self.start_point[0],
											self.start_point[1],
											self.start_point[2],
											virtual_obs_tolerance,
											virtual_obs_radius,
											pose_status_goal=start_virtual_obs)

			self.goal_maneuver = Maneuver(self.goal_point[0],
											self.goal_point[1],
											self.goal_point[2],
											virtual_obs_tolerance,
											virtual_obs_radius,
											pose_status_goal=goal_virtual_obs)   

	def make_maneuver(self, x, y):
		""" ."""

		if not self.virtual_obstacles:
			return True
		else:
			if self.start_maneuver.is_this_point_allowed(x, y) and self.goal_maneuver.is_this_point_allowed(x, y):
				return True
			else:
				return False

	def get_nodes_length(self):
		""" ."""
		return len(self.nodes)

	def choose_parent(self, new_node, parent):
		""" ."""
		for node in self.nodes:
			# distance node to new_node
			d = dist(node.point, new_node.point)

			# connection cost node to new_node
			cost_n_2_new_n = node.cost + dist(node.point, new_node.point)

			# connection cost node to parent
			cost_p_2_new_n = parent.cost + dist(parent.point, new_node.point)

			if d < self.optimization_radius and cost_n_2_new_n < cost_p_2_new_n:
				if self.obstacle_free(node.point, new_node.point):
					parent = node

		new_node.cost = parent.cost + dist(parent.point, new_node.point)
		new_node.parent = parent
		return new_node

	def rewire(self, new_node):
		""" ."""
		for node in self.nodes:
			# new_node parent
			new_n_p = new_node.parent

			# distance node to new_node
			d = dist(node.point, new_node.point)

			# connection cost node to new_node
			cost_new_n_2_n = new_node.cost + dist(node.point, new_node.point)

			if node != new_n_p and d < self.optimization_radius and cost_new_n_2_n < node.cost:
				if self.obstacle_free(node.point, new_node.point):
					# Now the node parent is the new node
					node.parent = new_node
					node.cost = new_node.cost + dist(node.point, new_node.point)				

	def grow_tree(self, random_sample=True, samples_per_beacon=5):
		""" ."""
		found_next = False
		if random_sample:
			while found_next == False:
				p_rand = self.sample_free()
				if self.found_next_node(p_rand):
					found_next = True
		else:
			# Intelligent Sampling only the intermediate beacons
			start = 1
			end = (len(self.beacons) - 1)
			for i in range(samples_per_beacon):
				for beacon in self.beacons[start: end]:
					found_next = False
					while found_next == False:
						p_rand = self.sample_free(random_sample=False, beacon_point=beacon.point)
						if self.found_next_node(p_rand):
							found_next = True

	def found_next_node(self, random_point):
		""" ."""
		n_nearest = self.get_nearest(random_point)
		p_new = self.steer(n_nearest.point, random_point)
		if self.obstacle_free(n_nearest.point, p_new):
			self.insert_node(p_new, n_nearest)
			return True
		else:
			return False
		
	def insert_node(self, p_new, n_nearest):
		""" ."""
		parent_node = n_nearest
		new_node = Node(p_new, parent_node)

		new_node = self.choose_parent(new_node, parent_node)
		self.nodes.append(new_node)

		self.nodes[-1].id = len(self.nodes) - 1

		self.rewire(new_node)
	            
		self.new_node = self.nodes[-1]

	def sample_free(self, random_sample=True, beacon_point=None):
		"""  Get a random point located in a free area
		random.random() returns a random number between 0 and 1
		point_rand = RANDOM_NUMBER * XDIM, RANDOM_NUMBER * YDIM
		"""
		for i in range(1000):
			if not random_sample:
				x_rand = beacon_point[0] + self.biasing_radius * 2 * (random.random() - 0.5)
				y_rand = beacon_point[1] + self.biasing_radius * 2 * (random.random() - 0.5)
			else:
				x_rand = (random.random() - 0.5) * self.x_dim 
				y_rand = (random.random() - 0.5) * self.y_dim 
				
			p_rand = x_rand, y_rand

			# Check collision
			if not self.collision(p_rand):
				return p_rand

		sys.exit("ERROR MESSAGE: Samples in free space fail after 1000 attempts!!!")

	def steer(self, p1, p2):
		""" ."""
		distance = dist(p1,p2)
		if distance < self.epsilon:
			return p2
		else:
			theta = math.atan2(p2[1]-p1[1],p2[0]-p1[0])
			return int(p1[0] + self.epsilon*math.cos(theta)), int(p1[1] + self.epsilon*math.sin(theta))

	def get_nearest(self, p_rand):
		""" Returns the nearest node of the list."""
		n_nearest = self.nodes[0]
		for node in self.nodes:
			if dist(node.point, p_rand) < dist(n_nearest.point, p_rand):
				n_nearest = node

		return n_nearest

	def get_new_node(self):
		""" Returns the new node of the tree."""
		return self.new_node

	def attempt_connect(self, external_node):
		""" Attempt to connect an external node in this tree."""
		n_nearest = self.get_nearest(external_node.point)

		if dist(n_nearest.point, external_node.point) < self.goal_tolerance:
			# check collision
			if self.obstacle_free(n_nearest.point, external_node.point):
				self.n_nearest_ext = n_nearest
				return True
		return False

	def update_radius(self):
		""" Update the radius of the algorithm optimization area."""
		nodes_size = len(self.nodes) + 1
		self.radius = self.k*math.sqrt((math.log(nodes_size) / nodes_size))

	def get_external_nodes(self, n_nearest_ext):
		""" Get the nodes that lead to the initial node of the tree,
		    starting from the nearest node of the external node
			provided (n_nearest_ext)."""
		external_nodes = list()
		current_node = n_nearest_ext
		while current_node.parent != None:
			external_nodes.append(current_node)
			current_node = current_node.parent

		# Add the first tree node
		external_nodes.append(current_node)

		return external_nodes

	def get_n_nearest_external(self):
		""" Returns the node of this tree that is the nearest node
		    of the new node of the other tree."""
		return self.n_nearest_ext

	def add_nodes_to_tree(self, external_nodes, parent_node):
		""" Add a set of external nodes to this tree."""
		current_parent = parent_node

		for node in external_nodes:
			node.parent = current_parent
			node.cost = current_parent.cost + dist(node.point, current_parent.point)
			
			current_parent = node

			# add to the nodes list
			self.nodes.append(node)

			# Update node id
			id = len(self.nodes) - 1
			self.nodes[len(self.nodes) - 1].id = id

		# Set New Goal
		self.goal = self.nodes[len(self.nodes) - 1]
	
	def block_tree(self):
		""" This tree does not grow anymore."""
		self.tree_blocked = True

	def compute_path(self):
		""" Compute the current path resultant of the
		    fusion of the two trees.."""
		path = list()
		current_node = self.goal

		self.beacons = list()

		while current_node.parent != None:
			path.insert(0, current_node.point)
			self.beacons.insert(0, current_node)

			current_node = current_node.parent

		# Add the start point
		path.insert(0, current_node.point)
		self.beacons.insert(0, current_node)

		if not self.is_start_tree_the_last():
			path.reverse()
		
		return path

	def is_tree_blocked(self):
		""" Returns true if this tree is blocked, false otherwise."""
		return self.tree_blocked

	def set_goal(self, goal_node):
		""" Set goal node. Necessary for RRT* that has just one Tree growing."""
		self.goal = goal_node

	def get_goal(self):
		""" Get goal node."""
		return self.goal

	def path_optimization(self):
		""" RRT*-Smart algorithm Path Optimization Step."""
		current_node = self.goal

		while current_node.parent.parent != None:
			if self.obstacle_free(current_node.point, current_node.parent.parent.point):
				# Update parent node
				current_node.parent = current_node.parent.parent
				self.nodes[current_node.id].parent = current_node.parent

				# Update current node cost
				parent_cost = current_node.parent.cost
				dist_cost = dist(current_node.parent.point, current_node.point)
				self.nodes[current_node.id].cost = parent_cost + dist_cost
			
			if current_node.parent.parent == None:
				break
			current_node = current_node.parent
	
		return self.compute_path()

	def collision(self, p):  # check if point collides with the obstacle
		""" Check if the point p is located in a occupied cell."""
		cell_row, cell_col = self.world_to_map(p[0], p[1])

		if type(self.grid) == np.ndarray:
			if cell_col < 0:
				return True
			elif cell_row < 0:
				return True
			elif cell_row - 1 < 0:
				return True
			elif cell_col - 1 < 0:
				return True
			elif cell_col >= self.grid.shape[1]:
				return True
			elif cell_row >= self.grid.shape[0]:
				return True
			elif cell_col + 1 >= self.grid.shape[1]:
				return True
			elif cell_row +1>= self.grid.shape[0]:
				return True
			elif self.grid[cell_row][cell_col] == 1:
				return True
			elif self.grid[cell_row+1][cell_col+1] == 1:
				return True
			elif self.grid[cell_row][cell_col+1] == 1:
				return True
			elif self.grid[cell_row+1][cell_col] == 1:
				return True
			elif self.grid[cell_row-1][cell_col] == 1:
				return True
			elif self.grid[cell_row][cell_col-1] == 1:
				return True
			elif self.grid[cell_row-1][cell_col-1] == 1:
				return True
			else:
				return False
		else:
			# ROS nav_msgs/OccupancyGrid Message
			index = cell_col + self.grid.info.width*cell_row

			if cell_col < 0:
				return True
			elif cell_row < 0:
				return True
			elif cell_row - 1 < 0:
				return True
			elif cell_col -1 < 0:
				return True
			elif cell_col >= self.grid.info.width:
				return True
			elif cell_row >= self.grid.info.height:
				return True
			elif self.grid.data[index] > 1:
				return True
			else:
				return False

	def world_to_map(self, x, y):
		""" get the cell coord of the center point of the robot"""
		# TODO Improve
		if type(self.grid) == np.ndarray:
			cell_col = int((x - (-5.0)) / 0.2)
			cell_row = int((-y - (-5.0)) / 0.2) 
		else:
			# ROS nav_msgs/OccupancyGrid Message
			cell_col = int(round((x - self.grid.info.origin.position.x) / self.grid.info.resolution))
			cell_row = int(round((y - self.grid.info.origin.position.y) / self.grid.info.resolution))

		return cell_row, cell_col

	def step_n_from_p1_to_p2(self, p1, p2, n):
		""" ."""
		theta = math.atan2(p2[1]-p1[1], p2[0]-p1[0])
		return (p1[0] + n*self.obs_resolution*math.cos(theta),
				p1[1] + n*self.obs_resolution*math.sin(theta))
	
	def obstacle_free(self, p1, p2):
		""" Check if there is an obstacle between points p1 and p2."""
		distance = dist(p1, p2)
		n = 1
		if distance < self.obs_resolution:
			if self.collision(p2):
				return False
			else:
				return True
		else:
			for i in range(int(math.floor(distance/self.obs_resolution))):
				p_i = self.step_n_from_p1_to_p2(p1, p2, i + 1)
				if self.collision(p_i) or not self.make_maneuver(p_i[0], p_i[1]):
					return False

			return True

	def is_start_tree_the_last(self):
		""" ."""
		return self.is_start_tree

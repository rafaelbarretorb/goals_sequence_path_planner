#!/usr/bin/env python

import random
import math
import numpy as np

from maneuver_bubble import Maneuver

import sys


class RRT_Star:
    """
        Class for RRT* Path Planning
    """
    def __init__(self, start_point, goal_point, grid, min_num_nodes, max_num_nodes,
                 epsilon_min, epsilon_max, radius, goal_tolerance, obs_resolution, maneuvers=False):

        self.grid = grid
        self.start_point = start_point[:]
        self.goal_point = goal_point[:]

        self.min_num_nodes = min_num_nodes
        self.max_num_nodes = max_num_nodes
        self.epsilon_min = epsilon_min
        self.epsilon_max = epsilon_max
        self.obs_resolution = obs_resolution

        self.maneuvers = maneuvers

        self.radius = radius

        if self.collision(start_point):
            print start_point
            sys.exit("ERROR!!!!. Start Position is not allowed.")

        if self.collision(goal_point):
            print goal_point
            sys.exit("ERROR!!!!. Goal Position is not allowed.")

        if self.maneuvers:
            self.start_maneuver = Maneuver(self.start_point[0], self.start_point[1], self.start_point[2], 0.05, 0.5, pose_status_goal=False)
            self.goal_maneuver = Maneuver(self.goal_point[0], self.goal_point[1], self.goal_point[2], 0.2, 0.5, pose_status_goal=True)     

        self.goal_tolerance = goal_tolerance
        self.node_id = -1

        self.goal_found = False
        self.goal_node_id = None

    def add_new_node(self, new_node):
        # Insert the first node into the nodes array
        self.nodes = np.append(self.nodes, new_node, axis=1)

    def make_maneuver(self, x, y):

        if not self.maneuvers:
            return True
        else:
            if self.start_maneuver.is_this_point_allowed(x, y) and self.goal_maneuver.is_this_point_allowed(x, y):
                return True
            else:
                return False

    def get_vector_distance(self, point):
        """ Return numpy array of the distance of between all nodes and the given point."""
        # inner function
        def diff_square_func(m):
            return (m - v)*(m - v)

        # Vectorize the function
        diff_square_vec = np.vectorize(diff_square_func)

        # x array
        v = point[0]
        x_arr = self.nodes[0, :]
        x_diff2_arr = diff_square_vec(x_arr)

        # y array
        v = point[1]
        y_arr = self.nodes[1, :]
        y_diff2_arr = diff_square_vec(y_arr)

        dist_arr = np.sqrt(x_diff2_arr + y_diff2_arr)

        return dist_arr

    def get_nearest(self, p_rand):
        """ Return the node_id of the nearest node."""
        # return the min index
        return np.argmin(self.get_vector_distance(p_rand))

    def make_node(self, x, y, parent_id):
        """ 
                 x
                 y
        node =   cost
                 node_id
                 parend_id
        
        """
        return np.array([[x], [y], [0.0], [self.make_node_id()], [parent_id]])

    def make_node_id(self):
        self.node_id = self.node_id + 1
        return self.node_id

    def dist(self, px1, py1, px2, py2):    
        """ Class method for compute the distance between two points.

        Args:
            p1: Point 1 tuple.
            p2: Poinf 2 tuple.

        Returns:
            The distance between two points the in cartesian plan.

        """
        distance = math.sqrt((px1-px2)*(px1-px2) + (py1-py2)*(py1-py2))
        return distance

    def choose_parent(self, new_node):

        # distance array
        #print ""
        dist_arr = self.get_vector_distance([new_node[0], new_node[1]])


        dist_arr_numbered = np.stack((dist_arr, np.arange(dist_arr.shape[0])), axis = 0)

        condition = dist_arr_numbered[0, :] < self.radius

        curr_parent_id = int(new_node[4][0])
        x_new_node = new_node[0][0]
        y_new_node = new_node[1][0]

        nodes_ids_selected = dist_arr_numbered[:, condition][1]
        for node_id in nodes_ids_selected:
            cost_node = self.nodes[2][int(node_id)]
            x_node = self.nodes[0][int(node_id)]
            y_node = self.nodes[1][int(node_id)]

            cost_curr_parent = self.nodes[2][curr_parent_id]
            x_curr_parent = self.nodes[0][curr_parent_id]
            y_curr_parent = self.nodes[1][curr_parent_id]

            if cost_node + self.dist(x_node, y_node, x_new_node, y_new_node) < cost_curr_parent + self.dist(x_curr_parent, y_curr_parent, x_new_node, y_new_node):
                if self.obstacle_free([x_node, y_node], [x_new_node, y_new_node]):
                    curr_parent_id = int(node_id)
        
        # Update the cost and parent_id of the new node
        # cost
        new_node[2] = self.nodes[2][curr_parent_id] + self.dist(self.nodes[0][curr_parent_id], self.nodes[1][curr_parent_id], new_node[0], new_node[1])
        # parent_id
        new_node[4] = curr_parent_id
        return new_node

    def rewire(self, new_node_id):

        new_node_id = int(new_node_id)
        # distance array
        dist_arr = self.get_vector_distance([self.nodes[0][int(new_node_id)], self.nodes[1][int(new_node_id)]])
        dist_arr_numbered = np.stack((dist_arr, np.arange(dist_arr.shape[0])), axis = 0)

        new_node_parent_id = self.nodes[4][new_node_id]

        condition = dist_arr_numbered[0, :] < self.radius
        cost_new_node = self.nodes[2][new_node_id]
        x_new_node = self.nodes[0][new_node_id]
        y_new_node = self.nodes[1][new_node_id]

        nodes_ids_selected = dist_arr_numbered[:, condition][1]
        for node_id in nodes_ids_selected:
            x_node = self.nodes[0][int(node_id)]
            y_node = self.nodes[1][int(node_id)]
            cost_node = self.nodes[2][int(node_id)]
            if int(node_id) != new_node_parent_id and cost_new_node + self.dist(x_node, y_node, x_new_node, y_new_node) < cost_node:
                if self.obstacle_free([x_node, y_node], [x_new_node, y_new_node]):
                    # Update the cost and parent_id of the new node
                    self.nodes[4][int(node_id)] = new_node_id
                    self.nodes[2][int(node_id)] = cost_new_node + self.dist(x_node, y_node, x_new_node, y_new_node)

    def path_planning(self):
        """ Execute the planner"""

        # First node
        initial_node = self.make_node(self.start_point[0], self.start_point[1], -1)

        self.nodes = np.copy(initial_node)

        while self.nodes.shape[1] < self.max_num_nodes:
            foundNext = False
            
            # search a node until get one in free space
            while foundNext == False:
                p_rand = self.sample_free() # random point in the free space
                p_nearest_id = self.get_nearest(p_rand) # return the nearest node id
                p_new = self.steer([self.nodes[0][p_nearest_id], self.nodes[1][p_nearest_id]], p_rand)
                if self.obstacle_free([self.nodes[0][p_nearest_id], self.nodes[1][p_nearest_id]], p_new):
                    parent_node_id = p_nearest_id
                    new_node = self.make_node(p_new[0], p_new[1], parent_node_id)

                    new_node_id = new_node[3][0]

                    new_node = self.choose_parent(new_node)
                    self.add_new_node(new_node)
                    self.rewire(new_node_id)            
                    foundNext = True

            if not self.goal_found:
                # check if the distance between the goal node and the new node is less than the goal tolerance
                
                if self.is_goal_reached(int(new_node_id), self.goal_point, self.goal_tolerance):
                    self.goal_found = True

                    # Set the goal node identification
                    self.goal_node_id = int(self.nodes[3][-1])
                    path_x, path_y = self.compute_path()
                    if self.nodes.shape[1] >= self.min_num_nodes:
                        print "Nodes Amount: " + str(self.nodes.shape[1])
                        return path_x, path_y
            else:
                path_x, path_y = self.compute_path()
            
                if self.nodes.shape[1] >= self.min_num_nodes:
                    print "Nodes Amount: " + str(self.nodes.shape[1])
                    return path_x, path_y
        
        return [], []

    def compute_path(self):
        """ . """
        path_x = list()
        path_y = list()

        curr_node_id = self.goal_node_id
        while curr_node_id != -1:
            path_x.insert(0, self.nodes[0][curr_node_id])
            path_y.insert(0, self.nodes[1][curr_node_id])

            parent_id = int(self.nodes[4][curr_node_id])
            curr_node_id = parent_id          

        return path_x, path_y

    def sample_free(self):
        """  Get a random point located in a free area

        random.random() returns a random number between  0 and 1

        x_rand = (RANDOM_NUMBER - MAX_RANDOM_NUMBER/2)*XDIM, random.random()*self.YDIM

        """
        for i in range(10000):
            x_rand = (random.random() - 0.5)*16, (random.random() - 0.5)*16
            if not self.collision(x_rand) and self.make_maneuver(x_rand[0], x_rand[1]):
                return x_rand

        sys.exit("ERROR MESSAGE: Samples in free space fail after 1000 attempts!!!")

    def world_to_map(self, x, y):
        """ get the cell coord of the center point of the robot"""
        if type(self.grid) == np.ndarray:
            cell_col = int((x - (-5.0)) / 0.2)
            cell_row = int((-y - (-5.0)) / 0.2) 
        else:
            # ROS nav_msgs/OccupancyGrid Message
            cell_col = int(round((x - self.grid.info.origin.position.x) / self.grid.info.resolution))
            cell_row = int(round((y - self.grid.info.origin.position.y) / self.grid.info.resolution))

        return cell_row, cell_col

    def is_goal_reached(self, new_node_id, goal_point, radius):
        """ ."""
        new_node_id = int(new_node_id)
        distance = self.dist(self.nodes[0][new_node_id], self.nodes[1][new_node_id], goal_point[0], goal_point[1])
        if (distance <= radius):
            return True
        return False

    def steer(self, p1, p2):
        """ ."""
        distance = self.dist(p1[0], p1[1], p2[0], p2[1])
        if distance < self.epsilon_max and distance > self.epsilon_min:
            return p2
        else:
            theta = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
            return p1[0] + self.epsilon_max*math.cos(theta), p1[1] + self.epsilon_max*math.sin(theta)

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

    def step_from_to2(self, p1, p2, n):
        # TODO Rename this method
        """ ."""
        theta = math.atan2(p2[1]-p1[1], p2[0]-p1[0])
        return (p1[0] + n*self.obs_resolution*math.cos(theta),
                p1[1] + n*self.obs_resolution*math.sin(theta))

    def obstacle_free(self, point1, point2):
        """ ."""
        distance = self.dist(point1[0], point1[1], point2[0], point2[1])
        n = 1
        if distance < self.obs_resolution:
            if self.collision(point2) or not self.make_maneuver(point2[0], point2[1]):
                return False
            else:
                return True
        else:
            for i in range(int(math.floor(distance/self.obs_resolution))):
                x_n = self.step_from_to2(point1, point2, n)
                if self.collision(x_n) or not self.make_maneuver(x_n[0], x_n[1]):
                    return False
                n = n + 1

            return True

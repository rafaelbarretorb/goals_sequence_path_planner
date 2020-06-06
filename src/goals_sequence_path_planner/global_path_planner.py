#!/usr/bin/env python

from scipy import interpolate

from rrt_star import RRT_Star
from helper_functions import dist, aim_to_point

import numpy as np
import matplotlib.pyplot as plt
import pylab
import math

PI = math.pi

class GlobalPathPlanner:
    """
    """

    def __init__(self, start_pose, global_map, final_pose):

        self.start_pose = start_pose
        self.final_pose = final_pose
        self.global_map = global_map

    def global_path_planner(self, goals_list):

        # paths with standard method
        usual_paths = self.usual_paths(goals_list)

        # compute the goals orientation
        for i in range(len(goals_list)):
            yaw, ideal_yaw = self.get_goal_orientation(usual_paths[i], usual_paths[i+1])
            goals_list[i].append(yaw)

        points = goals_list[:]

        points.insert(0, self.start_pose)
        points.append(self.final_pose)
        # paths with proposed method
        optimal_paths = self.optimized_paths(points)

        # smoother paths
        smoother_paths = self.make_paths_smoother(optimal_paths)

        return smoother_paths


    def usual_paths(self, goals_list):
        
        usual_paths = list()

        points = goals_list[:]

        # Insert the start point
        points.insert(0, [self.start_pose[0], self.start_pose[1]])
        
        # Insert the final point
        points.insert(len(points), [self.final_pose[0], self.final_pose[1]])

        # Compute all usual paths
        for i in range(len(points) - 1):
            path = list()
            # make RRT* Path Planning
            rrt_star = RRT_Star(start_point=points[i], goal_point=points[i+1], grid=self.global_map,
                max_num_nodes=5000,
                epsilon_min=0.0,
                epsilon_max=0.5, obs_resolution=0.1, maneuvers=False)

            path_x, path_y = rrt_star.path_planning()
            
            # Change the goal positions
            # if i < len(points) - 1:
            #     points[i+1][0] = path_x[-2]
            #     points[i+1][1] = path_y[-2]

            for j in range(len(path_x)):
                path.append([path_x[j], path_y[j]])

            usual_paths.append(path)
        
        return usual_paths

    def optimized_paths(self, points):
        
        optimal_paths = list()

        # Compute all optimized paths
        for i in range(len(points) - 1):
            path = list()
            # make RRT* Path Planning
            rrt_star = RRT_Star(start_point=points[i], goal_point=points[i+1], grid=self.global_map,
                max_num_nodes=5000,
                epsilon_min=0.0,
                epsilon_max=0.5, obs_resolution=0.1, maneuvers=True)

            path_x, path_y = rrt_star.path_planning()
            
            if len(path_x) != 0:
                # Change the goal positions
                if i < len(points) - 1:
                    points[i+1][0] = path_x[-1]
                    points[i+1][1] = path_y[-1]

                for j in range(len(path_x)):
                    path.append([path_x[j], path_y[j]])

                optimal_paths.append(path)
            else:
                optimal_paths.append([])

        
        return optimal_paths

    def departure_angle(self, first_x, first_y, second_x, second_y):
        """ return the start angle"""
        return aim_to_point(first_x, first_y, second_x, second_y)
    
    def arrival_angle(self, last_but_one_x, last_but_one_y, last_x, last_y):
        """ return the final angle of the last pose of the path"""
        return aim_to_point(last_but_one_x, last_but_one_y, last_x, last_y)
               
    def angle_distance(self, theta1, theta2): 
        """."""
        return math.atan2(math.sin(theta1 - theta2), math.cos(theta1 - theta2))

    def get_goal_orientation(self, path1, path2):
        """."""
        theta1 = self.arrival_angle(path1[-2][0], path1[-2][1], path1[-1][0], path1[-1][1]) 
        theta2 = self.departure_angle(path2[0][0], path2[0][1], path2[1][0], path2[1][1])
        dist = self.angle_distance(theta1, theta2)

        if (abs(dist) > PI/2):
            dist = np.sign(dist)*PI/2
            return (theta1 - dist), theta2
        else:
            return theta2, theta2

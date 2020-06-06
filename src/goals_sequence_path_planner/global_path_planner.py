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

    # TODO
    # - Risk Zones?
    def __init__(self, start_pose, global_map, final_pose):

        self.start_pose = start_pose
        self.final_pose = final_pose
        self.global_map = global_map

    def global_path_planner(self, goals_list):

        # paths with standard method
        paths = self.usual_paths(goals_list)

        dod_angles = list()
        doa_angles = list()
        goals_angles = list()


        # compute the goals orientation
        for i in range(len(goals_list)):
            dod, doa = self.get_goal_orientation(paths[i], paths[i+1])
            dod_angles.append(dod)
            doa_angles.append(doa)
            goals_angles.append(goals_list[i][2])

        goals_angles = self.get_best_angle(dod_angles, goals_angles)

        for i in range(len(goals_angles)):
            goals_list[i][2] = goals_angles[i]

        points = copy.deepcopy(goals_list)
        points.insert(0, self.start_pose)
        points.append(self.final_pose)

        # paths with proposed method
        optimal_paths = self.optimized_paths(points)

        # check if there is an empty path
        for i in range(len(optimal_paths)):
            if len(optimal_paths[i]) == 0:
                optimal_paths[i] = copy.deepcopy(paths[i])

        return optimal_paths


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
                max_num_nodes=10000,
                epsilon_min=0.1,
                epsilon_max=0.5, obs_resolution=0.1, maneuvers=False)

            path_x, path_y = rrt_star.path_planning()
            
            # Change the goal positions
            if i < len(points) - 1:
                points[i+1][0] = path_x[-1]
                points[i+1][1] = path_y[-1]

            for j in range(len(path_x)):
                path.append([path_x[j], path_y[j]])

            usual_paths.append(path)
        
        return usual_paths

    def optimized_paths(self, points):
        
        optimal_paths = list()

        last_path_failed = False

        # Compute all optimized paths
        for i in range(len(points) - 1):
            if points[i][2] == 10000:
                optimal_paths.append([])
            else:

                path = list()

                rrt_star = RRT_Star(start_point=points[i], goal_point=points[i+1], grid=self.global_map,
                    max_num_nodes=10000,
                    epsilon_min=0.1,
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
                    last_path_failed = False
                else:
                    last_path_failed = True
                    optimal_paths.append([])
        
        return optimal_paths

    def departure_angle(self, first_x, first_y, second_x, second_y):
        """ return the start angle"""
        return aim_to_point(first_x, first_y, second_x, second_y)
    
    def arrival_angle(self, last_but_one_x, last_but_one_y, last_x, last_y):
        """ return the final angle of the last pose of the path"""
        return aim_to_point(last_but_one_x, last_but_one_y, last_x, last_y)
               
    def angle_distance(self, x, y): 
        """."""
        if x < 0:
            x = x + 2*PI
        
        if y < 0:
            y = y + 2*PI
        
        a = x - y
        # if a > PI:
        #     a = a - 2*PI
        
        # if a < -PI:
        #     a = a + 2*PI 

        a = (a + PI) % (2*PI) - PI

        #return min(y-x, y-x+2*PI, y-x-2*PI)
        return a

    def get_goal_orientation(self, path1, path2):
        """."""
        doa = self.arrival_angle(path1[-2][0], path1[-2][1], path1[-1][0], path1[-1][1]) 
        dod = self.departure_angle(path2[0][0], path2[0][1], path2[1][0], path2[1][1])
        #dist = self.angle_distance(theta1, theta2)

        return dod, doa
    
    def get_best_angle(self, dod_angles, goals_angles):

        for i in range(len(dod_angles)):
            # print ""
            # print "DOD: " + str(math.degrees(dod_angles[i]))
            # print "Goal Angle: " + str(math.degrees(goals_angles[i]))
            # print "Distance 1: " + str(abs(math.degrees(self.angle_distance(dod_angles[i], goals_angles[i]))))
            # print "Distance 2: " + str(abs(math.degrees(self.angle_distance(dod_angles[i], goals_angles[i] - PI))))
            # print ""
            if goals_angles[i] != None:
                if abs(self.angle_distance(dod_angles[i], goals_angles[i])) > abs(self.angle_distance(dod_angles[i], goals_angles[i] - PI)):
                    goals_angles[i] = goals_angles[i] - PI

        return goals_angles

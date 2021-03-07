#!/usr/bin/env python

from scipy import interpolate

from rrt_star import RRT_Star
from helper_functions import dist, aim_to_point

import numpy as np
import matplotlib.pyplot as plt
import pylab
import math

import copy

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

        # dod_angles = list()
        # doa_angles = list()
        goals_angles = list()

        dod_between_doa_angles = list()

        # compute the goals orientation
        for i in range(len(goals_list)):
            dod, doa = self.get_goal_orientation(paths[i], paths[i+1])
            # dod_angles.append(dod)
            # doa_angles.append(doa)

            dod_between_doa_angles.append(self.avg_angle_of_two_angles(dod, doa))

            goals_angles.append(goals_list[i][2])
        
        # TODO        
        goals_angles = self.get_best_angle(dod_between_doa_angles, goals_angles)

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
                min_num_nodes=1000, max_num_nodes=5000,
                epsilon_min=0.1, epsilon_max=0.5, radius=1.0, goal_tolerance = 0.2,
                obs_resolution=0.1, maneuvers=False)

            path_x, path_y = rrt_star.path_planning()
            
            # Change the goal positions
            if i < len(points) - 1:
                # print "i = " + str(i)
                # print "points length: " + str(len(points))
                # print "path length: " + str(len(path_x))
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
            # Do not attempt optimize
            # TODO ERROR, points[i][3] ???
            if points[i+1][3] == False:
                optimal_paths.append([])
            else:

                path = list()
                
                # TODO Limit time (max 10 secs example) 
                rrt_star = RRT_Star(start_point=points[i], goal_point=points[i+1], grid=self.global_map,
                    min_num_nodes=1000, max_num_nodes=5000,
                    epsilon_min=0.1, epsilon_max=0.5, radius=1.0, goal_tolerance = 0.2,
                    obs_resolution=0.1, maneuvers=True)

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
        theta = aim_to_point(first_x, first_y, second_x, second_y)
        # print "DOD = " + str(math.degrees(theta))
        return theta
    
    def arrival_angle(self, last_but_one_x, last_but_one_y, last_x, last_y):
        """ return the final angle of the last pose of the path"""
        theta = aim_to_point(last_but_one_x, last_but_one_y, last_x, last_y)
        # print "DOA = " + str(math.degrees(theta))
        return theta
               
    def angle_distance(self, theta, phi):
        """ ."""
        if theta < 0:
            while theta < 0:
                theta = theta + 2*PI

        if phi < 0:
            while phi < 0:
                phi = phi + 2*PI

        diff = theta - phi

        diff = (diff + PI) % (2*PI) - PI

        return abs(diff)

    def get_goal_orientation(self, path1, path2):
        """."""
        doa = self.arrival_angle(path1[-2][0], path1[-2][1], path1[-1][0], path1[-1][1]) 

        # DOD
        # print "DOD" + str((path2[0][0], path2[0][1])) + " --> " + str((path2[1][0], path2[1][1]))
        # print path2
        dod = self.departure_angle(path2[0][0], path2[0][1], path2[1][0], path2[1][1])
        #dist = self.angle_distance(theta1, theta2)

        return dod, doa
    
    def get_best_angle(self, avg_angles, goals_angles):

        for i in range(len(avg_angles)):
            # print ""
            # print "DOD: " + str(math.degrees(dod_angles[i]))
            # print "Goal Angle: " + str(math.degrees(goals_angles[i]))
            # print "Distance 1: " + str(math.degrees(self.angle_distance(dod_angles[i], goals_angles[i])))
            # print "Distance 2: " + str(math.degrees(self.angle_distance(dod_angles[i], goals_angles[i] + PI)))
            # print ""
            if goals_angles[i] != None:
                if self.angle_distance(avg_angles[i], goals_angles[i]) > self.angle_distance(avg_angles[i], goals_angles[i] - PI):
                    goals_angles[i] = goals_angles[i] + PI

        return goals_angles

    def avg_angle_of_two_angles(self, theta1, theta2):
        half_ang_dist = self.angle_distance(theta1, theta2)/2

        if abs(self.wrap_to_pi(theta1 + half_ang_dist) - self.wrap_to_pi(theta2 - half_ang_dist)) < 0.01:
            print "avg angle" + str((180/PI)*self.wrap_to_pi(theta1 + half_ang_dist))
            return self.wrap_to_pi(theta1 + half_ang_dist)
        elif abs(self.wrap_to_pi(theta1 - half_ang_dist) - self.wrap_to_pi(theta2 + half_ang_dist)) < 0.01:
            print "avg angle" + str((180/PI)*self.wrap_to_pi(theta1 - half_ang_dist))
            return self.wrap_to_pi(theta1 - half_ang_dist)
        else:
            return None

    def wrap_to_pi(self, angle):
            while angle > PI:
                angle -= 2 * PI
            while angle < -PI:
                angle += 2 * PI
            return angle 

#!/usr/bin/env python

from scipy import interpolate

from rrt_star_smart_dual_tree import RRT_Star
from helper_functions import dist, aim_to_point

import numpy as np
import matplotlib.pyplot as plt
import pylab
import math

import copy

PI = math.pi

class SequenceOfGoalsPlanner:
    """
    """

    def __init__(self, start_pose, final_pose, goals_list, global_map, x_dim, y_dim):

        self.start_pose = start_pose
        self.final_pose = final_pose
        self.goals_list = goals_list
        self.global_map = global_map

        self.x_dim = x_dim
        self.y_dim = y_dim

        self.dod_angles = list()
        self.doa_angles = list()
        self.result_directions = list()

        self.usual_paths = list()
        self.optimized_paths = list()

        self.run_planner()

    def run_planner(self):

        # paths usual method
        self.usual_planning()

        # TODO remove this variable
        goals_angles = list()

        # compute the goals orientation
        for i in range(len(self.goals_list)):
            dod, doa = self.get_goal_orientation(self.usual_paths[i], self.usual_paths[i+1])
            self.dod_angles.append(dod)
            self.doa_angles.append(doa)
            self.result_directions.append(self.avg_angle_of_two_angles(dod, doa))

            goals_angles.append(self.goals_list[i][2])
          
        new_goals_angles = self.get_best_angle(self.result_directions, goals_angles)

        for i in range(len(new_goals_angles)):
            self.goals_list[i][2] = new_goals_angles[i]

        points = copy.deepcopy(self.goals_list)
        points.insert(0, self.start_pose)
        points.append(self.final_pose)

        # paths with proposed method
        self.optimized_planning(points)

    def usual_planning(self):

        points = self.goals_list[:]

        # Insert the start point
        points.insert(0, [self.start_pose[0], self.start_pose[1]])
        
        # Insert the final point
        points.insert(len(points), [self.final_pose[0], self.final_pose[1]])

        # Compute all usual paths
        for i in range(len(points) - 1):
            path = list()
            # make RRT* Path Planning
            planner = RRTStarSmartDualTree(start_point=points[i],
                                           goal_point=points[i+1],
                                           grid=self.global_map,
                                           min_num_nodes=1000,
                                           max_num_nodes=2000,
                                           goal_tolerance=0.2,
                                           epsilon=0.5,
                                           optimization_radius=1.0
                                           obs_resolution=0.1,
				                           biasing_ratio=50,
                                           x_dimension=10.0,
                                           y_dimension=10.0,
                                           maneuvers=False)

            path_x, path_y = planner.path_planning()
            
            # Change the goal positions
            if i < len(points) - 1:
                # print "i = " + str(i)
                # print "points length: " + str(len(points))
                # print "path length: " + str(len(path_x))
                points[i+1][0] = path_x[-1]
                points[i+1][1] = path_y[-1]

            for j in range(len(path_x)):
                path.append([path_x[j], path_y[j]])

            self.usual_paths.append(path)
        

    def optimized_planning(self, points):
        

        last_path_failed = False

        # Compute all optimized paths
        for i in range(len(points) - 1):
            # Do not attempt optimize
            # TODO ERROR, points[i][3] ???
            if points[i+1][3] == False:
                self.optimized_paths.append([])
            else:
                path = list()
                # TODO Limit time (max 10 secs example) 
                planner = RRTStarSmartDualTree(start_point=points[i],
                                               goal_point=points[i+1],
                                               grid=self.global_map,
                                               min_num_nodes=1000,
                                               max_num_nodes=2000,
                                               goal_tolerance=0.2,
                                               epsilon=0.5,
                                               optimization_radius=1.0
                                               obs_resolution=0.1,
                                               biasing_ratio=50,
                                               x_dimension=10.0,
                                               y_dimension=10.0,
                                               maneuvers=True)

                path_x, path_y = planner.path_planning()

                if len(path_x) != 0:

                    # Change the goal positions
                    if i < len(points) - 1:
                        points[i+1][0] = path_x[-1]
                        points[i+1][1] = path_y[-1]

                    for j in range(len(path_x)):
                        path.append([path_x[j], path_y[j]])

                    self.optimized_paths.append(path)
                    last_path_failed = False
                else:
                    print "Path " + str(i) + " failed!"
                    last_path_failed = True
                    self.optimized_paths.append([])
        
        # check if there is an empty path
        for i in range(len(self.optimized_paths)):
            if len(self.optimized_paths[i]) == 0:
                self.optimized_paths[i] = copy.deepcopy(self.usual_paths[i])

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
            # print "avg angle" + str((180/PI)*self.wrap_to_pi(theta1 + half_ang_dist))
            return self.wrap_to_pi(theta1 + half_ang_dist)
        elif abs(self.wrap_to_pi(theta1 - half_ang_dist) - self.wrap_to_pi(theta2 + half_ang_dist)) < 0.01:
            # print "avg angle" + str((180/PI)*self.wrap_to_pi(theta1 - half_ang_dist))
            return self.wrap_to_pi(theta1 - half_ang_dist)
        else:
            return None

    def wrap_to_pi(self, angle):
            while angle > PI:
                angle -= 2 * PI
            while angle < -PI:
                angle += 2 * PI
            return angle

    def get_doa_angles(self):
        return self.doa_angles

    def get_dod_angles(self):
        return self.dod_angles
    
    def get_resultant_direction_angles(self):
        return self.result_directions
    
    def get_usual_paths(self):
        return self.usual_paths
    
    def get_optimized_paths(self):
        return self.optimized_paths

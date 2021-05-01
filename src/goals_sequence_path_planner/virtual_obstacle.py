

import numpy as np
import matplotlib.pyplot as plt 
import pylab

# class parameters x0,y0,yaw

PI = np.pi

# TODO the main method shall check if the random point is inside proibited area

class VirtualObstacle:
  def __init__(self, pose, goal_tolerance, maneuver_radius, pose_status_goal=True):
    self.x0 = pose[0]
    self.y0 = pose[1]
    self.yaw = pose[2]
    self.goal_tolerance = goal_tolerance
    self.maneuver_radius = maneuver_radius
    self.pose_status_goal = pose_status_goal

    self.no_maneuver = False

    if self.yaw != None:
      self.xc_left, self.yc_left = self.local_to_world_tf(0.0, self.goal_tolerance+self.maneuver_radius)
      self.xc_right, self.yc_right = self.local_to_world_tf(0.0, -(self.goal_tolerance+self.maneuver_radius))
    else:
      self.no_maneuver = True

    # self.xc_front, self.yc_front = self.local_to_world_tf(0.0, -(self.goal_tolerance+self.maneuver_radius))

    # Arc Radius TODO
    self.arc_radius = 0.4
  
  def local_to_world_tf(self, x_, y_): 
    x = x_*np.cos(self.yaw) - y_*np.sin(self.yaw) + self.x0
    y = x_*np.sin(self.yaw) + y_*np.cos(self.yaw) + self.y0

    return x, y

  def world_to_local_tf(self, x, y): 
    # TODO
    x_ = (x - self.x0)*np.cos(self.yaw) + (y - self.y0)*np.sin(self.yaw)
    y_ = (y - self.y0)*np.cos(self.yaw) - (x - self.x0)*np.sin(self.yaw)

    return x_, y_

  def collision(self, x, y):
  
    if self.no_maneuver == True:
      return True

    # Left Circle
    condition1 = not self.is_point_inside_of_bloc_circle(x,
                                                         y,
                                                         self.xc_left,
                                                         self.yc_left,
                                                         self.maneuver_radius)
    
    # Right Circle
    condition2 = not self.is_point_inside_of_bloc_circle(x,
                                                         y,
                                                         self.xc_right,
                                                         self.yc_right,
                                                         self.maneuver_radius)

    # Arc
    condition3 = not self.is_point_inside_of_bloc_arc(x, y)

    return (condition1 and condition2 and condition3)

  def is_point_inside_of_bloc_circle(self, x, y, xc, yc, R):
    """ ."""
    f = (x - xc)*(x - xc) + (y - yc)*(y - yc)
    if (f < R*R):
      return True
    else:
      return False

  def is_point_inside_of_bloc_arc(self, x, y):
    """ ."""
    # Circle with radius = arc_radius
    condition1 = self.is_point_inside_of_bloc_circle(x, y, self.x0, self.y0, self.arc_radius)
    
    # py shall be less than 0.0 at local coordinate
    x_, y_ = self.world_to_local_tf(x, y)

    if self.pose_status_goal:
      condition2 = x_ > 0.0
    else:
      condition2 = x_ < 0.0

    # Tolerance Circle
    condition3 = self.is_point_inside_of_bloc_circle(x, y, self.x0, self.y0, self.goal_tolerance)

    if (condition1 and condition2 and not condition3):
      return True
    
    return False

  def make_arc(self):
    step = 5*PI/180
    if self.pose_status_goal:
      start_yaw = -PI/2 
      final_yaw = start_yaw + PI
    else:
      start_yaw = PI/2 
      final_yaw = start_yaw + PI

    theta = np.arange(start_yaw, final_yaw+step, step)

    x = self.arc_radius*np.cos(theta)
    y = self.arc_radius*np.sin(theta)

    return self.local_to_world_tf(x,y)

  def make_circle(self, xc, yc, radius):
    step = 5*PI/180
    theta = np.arange(0,(2*PI + step), step)

    x = xc + radius*np.cos(theta)
    y = yc + radius*np.sin(theta)
    
    return x, y
  
  def make_center_circle(self):
    return self.make_circle(self.x0, self.y0, self.goal_tolerance)

  def make_right_circle(self):
    return self.make_circle(self.xc_right, self.yc_right, self.maneuver_radius)

  def make_left_circle(self):
    return self.make_circle(self.xc_left, self.yc_left, self.maneuver_radius)

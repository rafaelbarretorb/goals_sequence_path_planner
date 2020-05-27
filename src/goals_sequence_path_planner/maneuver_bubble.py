

import numpy as np
import matplotlib.pyplot as plt 
import pylab

# class parameters x0,y0,yaw

PI = np.pi

# TODO the main method shall check if the random point is inside proibited area

class Maneuver:
  def __init__(self, x0, y0, yaw, goal_tolerance, maneuver_radius, pose_status_goal=True):
    self.x0 = x0
    self.y0 = y0 
    self.yaw = yaw
    self.goal_tolerance = goal_tolerance
    self.maneuver_radius = maneuver_radius
    self.pose_status_goal = pose_status_goal

    self.xc_left, self.yc_left = self.local_to_world_tf(0.0, self.goal_tolerance+self.maneuver_radius)
    self.xc_right, self.yc_right = self.local_to_world_tf(0.0, -(self.goal_tolerance+self.maneuver_radius)) 

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

  def is_this_point_allowed(self, px, py):
    
    # Left Circle
    A = not self.isThePointInsideOfTheCircle(px, py, self.xc_left, self.yc_left, self.maneuver_radius)
    
    # Right Circle
    B = not self.isThePointInsideOfTheCircle(px, py, self.xc_right, self.yc_right, self.maneuver_radius)

    # Arc
    C = not self.isThePointInsideOfTheBlocArc(px, py)

    return (A and B and C)

  def isThePointInsideOfTheCircle(self, px, py, xc, yc, R):
    f = (px - xc)*(px - xc) + (py - yc)*(py - yc)
    if (f < R*R):
      return True
    else:
      return False

  def isThePointInsideOfTheBlocArc(self, px, py):

    # Circle with radius = arc_radius
    A = self.isThePointInsideOfTheCircle(px, py, self.x0, self.y0, self.arc_radius)
    
    # py shall be less than 0.0 at local coordinate
    px_, py_ = self.world_to_local_tf(px, py)

    if self.pose_status_goal:
      B = px_ > 0.0
    else:
      B = px_ < 0.0

    # Tolerance Circle
    C = self.isThePointInsideOfTheCircle(px, py, self.x0, self.y0, self.goal_tolerance)

    if A and B and not C:
      return True
    
    return False

  def makeArc(self):
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

  def makeCircle(self, xc, yc, radius):
    step = 5*PI/180
    theta = np.arange(0,(2*PI + step), step)

    x = xc + radius*np.cos(theta)
    y = yc + radius*np.sin(theta)
    
    return x, y
  
  def makeCenterCircle(self):
    return self.makeCircle(self.x0, self.y0, self.goal_tolerance)

  def makeRightCircle(self):
    return self.makeCircle(self.xc_right, self.yc_right, self.maneuver_radius)

  def makeLeftCircle(self):
    return self.makeCircle(self.xc_left, self.yc_left, self.maneuver_radius)

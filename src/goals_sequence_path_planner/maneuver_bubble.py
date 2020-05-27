

import numpy as np
import matplotlib.pyplot as plt 
import pylab

from orientation_filter import aim_to_next_position, get_arrows, get_arrow_pose

# TODO make a class
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

def main():

  goal_tolerance = 0.2
  maneuver_radius = 0.5

  # START Pose
  start_pose = [0.0, 0.0, 0.0]
  start_maneuver = Maneuver(start_pose[0], start_pose[1], start_pose[2], 0.05, maneuver_radius, False)

  x_start, y_start = start_maneuver.makeCenterCircle()
  x_start_right, y_start_right = start_maneuver.makeRightCircle()
  x_start_left, y_start_left = start_maneuver.makeLeftCircle()
  x_start_arc, y_start_arc = start_maneuver.makeArc()
  
  # GOAL Pose
  goal_pose = [2.0,3.5, -PI/2]
  goal_maneuver = Maneuver(goal_pose[0], goal_pose[1], goal_pose[2], goal_tolerance, maneuver_radius, True)

  x_goal, y_goal = goal_maneuver.makeCenterCircle()
  x_goal_right, y_goal_right = goal_maneuver.makeRightCircle()
  x_goal_left, y_goal_left = goal_maneuver.makeLeftCircle()
  x_goal_arc, y_goal_arc = goal_maneuver.makeArc()

  
  plt.figure(figsize=(12,12))

  # Plot Start
  plt.plot(x_start,y_start)
  plt.plot(x_start_right, y_start_right)
  plt.plot(x_start_left, y_start_left)
  plt.plot(x_start_arc, y_start_arc)

  # Plot Goal
  plt.plot(x_goal,y_goal)
  plt.plot(x_goal_right, y_goal_right)
  plt.plot(x_goal_left, y_goal_left)
  plt.plot(x_goal_arc, y_goal_arc)

  # plt.plot(x_left,y_left)
  # plt.plot(x_goal,y_goal)

  plt.ylim([-5,5])
  plt.xlim([-5,5])

  p1 = (-0.35, 0.0) # False
  p1 = (-0.35, 1.0) # False
  p1 = (-0.35, 1.2) # True
  p1 = (-0.35, -1.2) # True
  p1 = (-0.35, -1.0) # False
  p1 = (0.35, -1.0) # False
  p1 = (0.35, -1.3) # True
  p1 = (0.1, 0.1) # True
  #p2 = (0.15, -0.2)
  # p2 = (0.15, 0.2)

  p2 = (1.75, 3.2)

  print 'Does the BLACK point is allowed: ' + str(start_maneuver.is_this_point_allowed(p1[0], p1[1]))
  print 'Does the GREEN point is allowed: ' + str(goal_maneuver.is_this_point_allowed(p2[0], p2[1]))

  plt.scatter(p1[0], p1[1], color='black')
  plt.scatter(p2[0], p2[1], color='green')

  # Plot Goal
  goal_endx, goal_endy = get_arrow_pose(goal_pose[0], goal_pose[1], goal_pose[2], arrow_length=0.5)
  pylab.arrow(goal_pose[0], goal_pose[1], goal_endx, goal_endy, width=0.01, color='black')

  # Plot Goal
  start_endx, start_endy = get_arrow_pose(start_pose[0], start_pose[1], start_pose[2], arrow_length=0.5)
  pylab.arrow(start_pose[0], start_pose[1], start_endx, start_endy, width=0.01, color='red')

  plt.show()

if __name__ == "__main__":
  main()

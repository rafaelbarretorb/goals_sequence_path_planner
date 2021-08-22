#!/usr/bin/env python

import math
import numpy as np
from scipy import interpolate
import copy
from goals_sequence_path_planner.helper_functions import distance


def get_arrow_pose(theta, arrow_length=0.25):
	endx = arrow_length*math.cos(theta)
	endy = arrow_length*math.sin(theta)

	return endx, endy

def make_spline_curve(path, max_distance, tolerance, insert_cv=True):
	spline_curve = list()
	data = np.array(path)
	if insert_cv:
		data = insert_control_points(data, max_distance, tolerance)
	spline_curve = bspline(data, n=100, degree=3)

	return spline_curve, data
  
def bspline(cv, n=100, degree=3, periodic=False):
	""" Calculate n samples on a bspline

		cv :      Array ov control vertices
		n  :      Number of samples to return
		degree:   Curve degree
		periodic: True - Curve is closed
				False - Curve is open
	"""

	# If periodic, extend the point array by count+degree+1
	cv = np.asarray(cv)
	count = len(cv)

	if periodic:
		factor, fraction = divmod(count+degree+1, count)
		cv = np.concatenate((cv,) * factor + (cv[:fraction],))
		count = len(cv)
		degree = np.clip(degree,1,degree)

	# If opened, prevent degree from exceeding count-1
	else:
		degree = np.clip(degree,1,count-1)


	# Calculate knot vector
	kv = None
	if periodic:
		kv = np.arange(0-degree,count+degree+degree-1)
	else:
		kv = np.clip(np.arange(count+degree+1)-degree,0,count-degree)

	# Calculate query range
	u = np.linspace(periodic,(count-degree),n)


	# Calculate result
	return np.array(interpolate.splev(u, (kv,cv.T,degree))).T


def insert_control_points(cv, max_distance, tolerance):
	""" ."""
	cv_copy = copy.deepcopy(cv)
	count = 0
	for i in range(1, cv.shape[0]):
		x1 = cv[i-1][0]
		y1 = cv[i-1][1]

		x2 = cv[i][0]
		y2 = cv[i][1]

		dist = distance((x1, y1), (x2, y2))
		
		if dist > max_distance:
			n = int(dist/max_distance)
			if dist - n*max_distance < tolerance*max_distance:
				n = n - 1
			for j in range(n):
				count = count + 1
				x, y = steer((x1, y1), (x2, y2), max_distance, j + 1)
				row = np.array([x, y])
				cv_copy = np.insert(cv_copy, i - 1 + count, row, axis=0)

	return cv_copy

def steer(p1, p2, epsilon, n):
	""" ."""
	theta = math.atan2(p2[1]-p1[1],p2[0]-p1[0])
	return p1[0] + n * epsilon * math.cos(theta), p1[1] + n * epsilon * math.sin(theta)

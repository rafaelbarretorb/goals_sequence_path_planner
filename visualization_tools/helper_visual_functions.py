#!/usr/bin/env python

import math
import numpy as np
from scipy import interpolate

def get_arrow_pose(x, y, theta, arrow_length=0.25):
    endx = arrow_length*math.cos(theta)
    endy = arrow_length*math.sin(theta)

    return endx, endy

def make_paths_smoother(path):
	smoother_path = list()
	data = np.array(path)
	smoother_path = bspline(data,n=100,degree=3)

	return smoother_path
  
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

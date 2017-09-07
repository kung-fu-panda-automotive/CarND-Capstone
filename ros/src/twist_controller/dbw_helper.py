#!/usr/bin/env python
#-------------------------------------------------------------------------------
# Author: Kostas Oreopoulos <kostas.oreopoulos@gmail.com>
# Date:   28.08.17
#-------------------------------------------------------------------------------
"""
A collection of helper functions for dbw node
"""

from math import sqrt, cos, sin
import numpy as np
import tf

POINTS_TO_FIT = 10


def fit_polynomial(waypoints, degree):
    """fits a polynomial for given waypoints"""
    x_coords = [waypoint.pose.pose.position.x for waypoint in waypoints]
    y_coords = [waypoint.pose.pose.position.y for waypoint in waypoints]
    return np.polyfit(x_coords, y_coords, degree)


def get_euler(pose):
    """Returns the roll, pitch yaw angles from a Quaternion """
    return tf.transformations.euler_from_quaternion([pose.orientation.x,
                                                     pose.orientation.y,
                                                     pose.orientation.z,
                                                     pose.orientation.w])


def eucleidian_distance(x0, y0, x1, y1):
    """The Eucleidian distance between points (x0,y0) and (x1,y1)"""
    return sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2))


def distance2parabola(coefficients, x, y):
    """
    Calculates the distance of a point from a parabola defined
    by its polynomial coefficients.

    Args:
         coefficients (list) : Polynomial coefficients from higher to lower order
         x (float) : X-Coordinate of the point we want distance for
         y (float) : Y-Coordinate of the point we want distance for
         plot (bool) : If True, create a plot of the parabola, the point and the distance

    Returns:
        distance (float) : The distance of the point to the parabola
        left (int) : 1 if the point is on the left of the curve as we "walk" the curve
                     from negative to positive x's. -1 if on the right
    """
    a, b, c = coefficients
    p0 = 2 * a * a
    p1 = 3 * a * b
    p2 = 2 * a * c - 2 * a * y + b * b + 1
    p3 = b * c - b * y - x

    p = [p0, p1, p2, p3]

    roots = np.roots(p)
    # filter the real only root
    x_ = np.real(roots[np.isreal(roots)])[0]
    y_ = np.polyval(coefficients, x_)

    distance = eucleidian_distance(x, y, x_, y_)
    left = -1 if y_ < y else 1

    return distance, left, x_, y_


def shift_and_rotate_waypoints(pose, waypoints, points_to_use=None):
    """
    From a pose object transfrom a series of waypoints so that
    the origin is at the pose position and the orientation matches
    the yaw of the pose

    Args:
        pose (object) : A pose object
        waypoints (list) : A list of waypoint objects
        points_to_use (int) : How many points to use (None => all)

    Returns:
        x_coords (list): The transformed x-coordinates of waypoints
        y_coords (list): The transformed y-coordinates of waypoints
    """
    x_coords = []
    y_coords = []

    _, _, yaw = get_euler(pose)
    originX = pose.position.x
    originY = pose.position.y

    if points_to_use is None:
        points_to_use = len(waypoints)

    for i in range(points_to_use):

        shift_x = waypoints[i].pose.pose.position.x - originX
        shift_y = waypoints[i].pose.pose.position.y - originY

        x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)
        y = shift_x * sin(0 - yaw) + shift_y * cos(0 - yaw)

        x_coords.append(x)
        y_coords.append(y)

    return x_coords, y_coords


def is_waypoint_behind(pose, waypoint):
    """Take a waypoint and a pose , do a coordinate system transformation
    setting the origin at the position of the pose object and as x-axis
    the orientation of the z-axis of the pose

    Args:
        pose (object) : A pose object
        waypoints (object) : A waypoint object

    Returns:
        bool : True if the waypoint is behind the car False if in front

    """
    _, _, yaw = get_euler(pose)
    originX = pose.position.x
    originY = pose.position.y

    shift_x = waypoint.pose.pose.position.x - originX
    shift_y = waypoint.pose.pose.position.y - originY

    x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)

    if x > 0:
        return False
    return True


def cte(pose, waypoints):
    """
    From a pose object and a series of waypoints, calculate the distance
    (cross track error) of the point, defined by the pose, to the path,
    defined by the series of waypoints.

    Args:
        pose (object) : A pose object
        waypoints (list) : A list of waypoint objects

    Returns:
        cte (float) : the cross track error (signed)
    """
    x_coords, y_coords = shift_and_rotate_waypoints(
        pose, waypoints, POINTS_TO_FIT)
    coefficients = np.polyfit(x_coords, y_coords, 3)
    distance = np.polyval(coefficients, 5.0)

    return distance

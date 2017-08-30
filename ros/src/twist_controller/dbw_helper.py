#!/usr/bin/env python
#-------------------------------------------------------------------------------
# Author: Kostas Oreopoulos <kostas.oreopoulos@gmail.com>
# Date:   28.08.17
#-------------------------------------------------------------------------------
"""
A collection of helper functions for dbw node
"""
# pylint: disable=import-error
from math import sqrt, cos, sin
import numpy as np
import matplotlib.pyplot as plt
import tf

# pylint: disable=invalid-name
# pylint: disable=too-many-instance-attributes

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


def distance2parabola(coefficients, x, y, plot=False):
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
    #pylint: disable=too-many-locals
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

    if plot:
        plot_distance(coefficients, x, y, x_, y_)

    distance = eucleidian_distance(x, y, x_, y_)
    left = -1 if y_ < y else 1

    return distance, left, x_, y_


def plot_distance(coefficients, x_0, y_0, x_, y_):
    """
    Args:
        coefficients (list): the coefficients of the parabola
        x_0 (float): the x coordinate of the point outside the parabola we want distance
        y_0 (float): the y coordinate of the point outside the parabola we want distance
        x_ (float): the x coordinate of the point on the parabola closest to (x_0,y_0)
        y_ (float): the y coordinate of the point on the parabola closest to (x_0,y_0)
    """
    def f(x):
        """
        Evaluate polynomial
        """
        return np.polyval(coefficients, x)

    X = [x_, y_]

    x = np.linspace(-15, 15, 100)

    plt.plot(x, f(x), 'r-', label='f(x)')
    plt.plot(x_0, y_0, 'bo', label='point')
    plt.plot([x_0, X[0]], [y_0, X[1]], 'b-', label='shortest distance')
    delta = 2.0
    plt.plot([X[0], X[0] + delta],
             [X[1], X[1] + (2.0 * coefficients[0] * X[0] +
                            coefficients[1]) * delta],
             'g-', label='tangent')

    plt.axis('equal')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend(loc='best')
    plt.show()


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
    x_coords, y_coords = shift_and_rotate_waypoints(pose, waypoints, POINTS_TO_FIT)
    coefficients = np.polyfit(x_coords, y_coords, 3)
    distance = np.polyval(coefficients, 2.0)

    return distance


if __name__ == '__main__':

    # cf = [-8.34706486e-04, -5.37658333e-01, -4.88838535e+00]
    # print(distance2parabola(cf, 0, 0, plot=True))

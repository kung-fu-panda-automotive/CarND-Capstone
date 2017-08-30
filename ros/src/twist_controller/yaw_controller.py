#-------------------------------------------------------------------------------
# Author: Kostas Oreopoulos <kostas.oreopoulos@gmail.com>
# Date:   28.08.17
#-------------------------------------------------------------------------------

"""
A simple Yaw Controller
"""

from math import atan

class YawController(object):
    """
    A simple Yaw Controller

    Args:
         wheel_base (float) : The car wheel base
         steer_ratio (float) : The car steer ratio
         min_speed (float) : The min speed
         max_lat_accel (float) : The max lateral acceleration
         max_steer_angle (float) : The maximum steering angle

    """
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        #pylint: disable=too-many-arguments
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle

    def get_angle(self, radius):
        """
        Given the radius of the curve , return the steering angle
        Args:
            radius (float) : The radius of the curve the car is driving
        Returns:
            angle (float) : The steering angle
        """
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        """
        Given the target velocities (angular and linear) and the current velocity
        calculate the correct steering angle to achieve the correct angular velocity
        based on the radius of the road

        Args:
            linear_velocity (float) : the target linear velocity (x-axis)
            angular_velocity (float) : the target angular velocity (z-axis)
            current-velocity (float) : the current linear velocity (x-axis)
        """
        angular_velocity = current_velocity * angular_velocity / linear_velocity \
          if abs(linear_velocity) > 0. else 0.

        if abs(current_velocity) > 0.1:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity)
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

        return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) \
          if abs(angular_velocity) > 0. else 0.0

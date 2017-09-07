#-------------------------------------------------------------------------------
# Author: Kostas Oreopoulos <kostas.oreopoulos@gmail.com>
# Date:   28.08.17
#-------------------------------------------------------------------------------
"""
A speed pid controller based on torque for dbw node
"""

MAX_THROTTLE_TORQUE = 2000.0
MAX_BREAK_TORQUE = 20000.0


class SpeedController(object):
    """ A Speed Controller class """
    # pylint: disable=too-few-public-methods
    def __init__(self, vehicle_mass, wheel_radius, accel_limit, decel_limit):
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit

    def control(self, target_velocity, current_velocity, realization_time):
        """
        Given the velocity error and the time elapsed
        since previous step it returns the correct throttle brake combination

        Args:
             target_velocity (float) : the target velocity of the car
             current_velocity (float) : the current velocity the car
             realization_time (float): The time to make the transition

        Returns:
             throttle (float) , brake (float)
        """
        # calculate change that needs to take place
        error = target_velocity - current_velocity
        # calculate the acceleration based on the time
        # we need to make the change happen
        acceleration = error / realization_time
        # apply limits to acceleration
        if acceleration > 0:
            acceleration = min(self.accel_limit, acceleration)
        else:
            acceleration = max(self.decel_limit, acceleration)
        # calculate torque = M*acc*R
        torque = self.vehicle_mass * acceleration * self.wheel_radius
        throttle, brake = 0, 0
        if torque > 0:
            # throttle is the percent of max torque applied
            throttle, brake = min(1.0, torque / MAX_THROTTLE_TORQUE), 0.0
        else:
            # brake is the torque we need to apply
            throttle, brake = 0.0, min(abs(torque), MAX_BREAK_TORQUE)

        return throttle, brake

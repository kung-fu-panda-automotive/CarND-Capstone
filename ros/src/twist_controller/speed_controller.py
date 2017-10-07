#-------------------------------------------------------------------------------
# Author: Kostas Oreopoulos <kostas.oreopoulos@gmail.com>
# Date:   28.08.17
#-------------------------------------------------------------------------------
"""
A speed pid controller based on torque for dbw node
"""

GAS_DENSITY = 2.858


class SpeedController(object):
    """ A Speed Controller class """
    # pylint: disable=too-few-public-methods

    def __init__(self, vehicle_mass, wheel_radius, accel_limit,
                 decel_limit, brake_deadband, fuel_capacity,
                 max_acceleration):
        # pylint: disable=too-many-arguments
        self.vehicle_mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.wheel_radius = wheel_radius
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.brake_deadband = brake_deadband
        # max torque corresponding to 1.0 throttle
        self.max_acc_torque = self.vehicle_mass * max_acceleration * self.wheel_radius
        # max brake torque corresponding to deceleration limit
        self.max_brake_torque = self.vehicle_mass * abs(self.decel_limit) * self.wheel_radius


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

        throttle, brake = 0., 0.
        # no controls if we are in the deadband
        if abs(acceleration) < self.brake_deadband:
            return throttle, brake

        # calculate torque = M*acc*R
        torque = self.vehicle_mass * acceleration * self.wheel_radius

        if torque > 0:
            # throttle is the percent of max torque applied
            throttle, brake = min(1.0, torque / self.max_acc_torque), 0.0
        else:
            # brake is the torque we need to apply
            throttle, brake = 0.0, min(abs(torque), self.max_brake_torque)

        return throttle, brake

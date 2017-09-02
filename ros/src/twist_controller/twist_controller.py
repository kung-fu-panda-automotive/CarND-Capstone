#-------------------------------------------------------------------------------
# Author: Kostas Oreopoulos <kostas.oreopoulos@gmail.com>
# Date:   28.08.17
#-------------------------------------------------------------------------------

"""
Implement a simple car controller
"""
# pylint: disable=invalid-name

import rospy
import pid
import lowpass

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    """
    Initialize the Controller Object by passing a dictionary with required values

    Args:
         kwargs (dict) : A dictionary with required properties to intantiate the controller
         The required properties are:

         kwargs.min_acc : deacceleration limit
         kargws.max_acc : acceleration limit
         kargws.wheel_base : car wheel base
         kargws.steer_ratio : the steering ratio
         kargws.max_lat_acc : max lateral acceleration
         kargws.max_steer_angle : max steering angle

    """

    def __init__(self, **kwargs):
        mn = kwargs.get('min_acc', 0.0)  # decel_limit
        mx = kwargs.get('max_acc', 0.0)  # accel_limit
        # wb = kwargs.get('wheel_base', 0.0)  # wheel_base
        # sr = kwargs.get('steer_ratio', 0.0)  # steer_ratio
        # ml = kwargs.get('max_lat_acc', 0.0)  # max_lat_accel
        ms = kwargs.get('max_steer_angle', 0.0)  # max_steer_angle
        # min_speed = 0.0

        # create controllers
        self.throttle_pid = pid.PID(kp=1.0, ki=0.02, kd=0.0, mn=mn, mx=mx)
        self.steer_pid = pid.PID(kp=0.5, ki=0.001, kd=0.5, mn=-ms, mx=ms)

        # create lowpass filters
        self.throttle_filter = lowpass.LowPassFilter(tau=0.10, ts=0.90)
        self.steer_filter = lowpass.LowPassFilter(tau=0.10, ts=0.90)

        # init timestamp
        self.timestamp = rospy.get_time()

    def control(self, vel_error, cte, dbw_enabled):
        """
        Given then target linear and angular velocities, as well as the current velocity
        calculate the CTE and pass them to the corresponding PID's

        Args:
             vel_error (float) : Velocity Error
             cte (float): Cross Track Error
             dbw_enabled (bool) : If True: auto pilot is on. If False: Manual driving
        """
        new_timestamp = rospy.get_time()
        duration = new_timestamp - self.timestamp
        sample_time = duration + 1e-6  # to avoid division by zero

        self.timestamp = new_timestamp
        if dbw_enabled:
            # initialize values
            brake = 0.
            throttle = 0.
            # calculate new steering angle
            steering_angle = self.steer_pid.step(cte, sample_time)
            steering_angle = self.steer_filter.filt(steering_angle)
            # caclulate throttle and brake
            throttle = self.throttle_pid.step(vel_error, sample_time)
            throttle = self.throttle_filter.filt(throttle)
            # convert to the expected format
            if throttle < 0.:
                brake = throttle
                throttle = 0.
            return throttle, brake, steering_angle
        # dbw is not enabled (manual override) so resetting pid's and filters
        self.throttle_pid.reset()
        self.steer_pid.reset()
        self.throttle_filter.last_val = 0.0
        self.steer_filter.last_val = 0.0

        return 0.0, 0.0, 0.0

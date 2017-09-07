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

    def __init__(self, max_steer_angle):
        ms = max_steer_angle  # max_steer_angle

        # create controllers
        self.steer_pid = pid.PID(kp=0.5, ki=0.003, kd=0.25, mn=-ms, mx=ms)
        # create lowpass filters
        self.steer_filter = lowpass.LowPassFilter(tau=0., ts=1.)
        # init timestamp
        self.timestamp = rospy.get_time()

    def control(self, cte, dbw_enabled):
        """
        Given then target linear and angular velocities, as well as the current velocity
        calculate the CTE and pass them to the corresponding PID's

        Args:
             cte (float): Cross Track Error
             dbw_enabled (bool) : If True: auto pilot is on. If False: Manual driving
        """
        new_timestamp = rospy.get_time()
        duration = new_timestamp - self.timestamp
        sample_time = duration + 1e-6  # to avoid division by zero

        self.timestamp = new_timestamp
        if dbw_enabled:
            # filter (smooth) errors
            cte = self.steer_filter.filt(cte)
            # calculate new steering angle
            steering_angle = self.steer_pid.step(cte, sample_time)
            return steering_angle
        # dbw is not enabled (manual override) so resetting pid's and filters
        self.steer_pid.reset()
        self.steer_filter.last_val = 0.0

        return 0.0

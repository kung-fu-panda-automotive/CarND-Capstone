#-------------------------------------------------------------------------------
# Author: Kostas Oreopoulos <kostas.oreopoulos@gmail.com>
# Date:   28.08.17
#-------------------------------------------------------------------------------

"""
A simple PID controller
"""

# pylint: disable=invalid-name
# pylint: disable=too-many-instance-attributes
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    """
    A simple PID controller object
    """

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, mn=MIN_NUM, mx=MAX_NUM, lowpass_filter=None):
        # pylint: disable=too-many-arguments
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_int_val = self.last_error = 0.
        self.filter = lowpass_filter

    def reset(self):
        """
        Resets Accumulated Error
        """

        self.int_val = 0.0
        self.last_int_val = 0.0

        if self.filter is not None:
            self.filter.last_val = 0.0

    def step(self, error, sample_time):
        """
        Given the CTE (cross track error) and the time elapsed
        since previous step it return a new value

        Args:
             error (float) : Cross Track Error
             sample_time (float): The time since the previous calculation

        Returns:
             val (float) : The value that should be applied to the controller
        """

        self.last_int_val = self.int_val

        integral = self.int_val + error * sample_time
        derivative = (error - self.last_error) / sample_time

        if self.filter is not None:
            derivative = self.filter.filt(derivative)

        y = self.kp * error + self.ki * self.int_val + self.kd * derivative
        val = max(self.min, min(y, self.max))

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val

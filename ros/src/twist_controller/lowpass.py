#-------------------------------------------------------------------------------
# Author: xxx xxx <xxx@xxx.xxx>
# Date:   xx.xx.xx
#-------------------------------------------------------------------------------

"""
STUB
"""

class LowPassFilter(object):
    """
    STUB
    """
    def __init__(self, tau, ts):
        self.a = 1. / (tau / ts + 1.)
        self.b = tau / ts / (tau / ts + 1.)

        self.last_val = 0.
        self.ready = False

    def get(self):
        """STUB"""
        return self.last_val

    def filt(self, val):
        """
        STUB
        """
        if self.ready:
            val = self.a * val + self.b * self.last_val
        else:
            self.ready = True

        self.last_val = val
        return val

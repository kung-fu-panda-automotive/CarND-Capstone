#-------------------------------------------------------------------------------
# Author: xxx xxx <xxx@xxx.xxx>
# Date:   xx.xx.xx
#-------------------------------------------------------------------------------

"""
DUMMY DOCSTRING - TO BE iMPLEMENTED
"""

from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    #pylint: disable=too-few-public-methods
    """
    DUMMY DOCSTRING - TO BE IMPLEMENTED
    """
    def __init__(self):
        pass

    def get_classification(self, image):
        #pylint: disable=unused-argument
        #pylint: disable=no-self-use
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        return TrafficLight.UNKNOWN

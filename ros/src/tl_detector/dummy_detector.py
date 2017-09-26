#-------------------------------------------------------------------------------
# Author: Mithi Sevilla <mithi.sevilla@gmail.com>
# Author: Lukasz Janyst <lukasz@jany.st>
# Date:   26.09.2017
#-------------------------------------------------------------------------------

"""
A Dummy Traffic Lights Detector to be used for testing
"""

import sys
import rospy

from styx_msgs.msg import TrafficLightArray, TrafficLight
from detector import Detector

#-------------------------------------------------------------------------------
class DummyDetector(Detector):
    """Dummy Traffic Light Detector"""

    #---------------------------------------------------------------------------
    def __init__(self):
        Detector.__init__(self)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray,
                         self.traffic_cb, queue_size=1)

        self.tl_map_filled = False

    #---------------------------------------------------------------------------
    def traffic_cb(self, msg):
        """Determines nearest red traffic light ahead of vehicle"""

        #-----------------------------------------------------------------------
        # Initialize
        #-----------------------------------------------------------------------
        if self.base_waypoints is None or self.car_index is None:
            return

        if not self.tl_map_filled:
            lights = [x.pose.pose.position for x in msg.lights]
            self.fill_tl_map(lights)
            self.tl_map_filled = True

        #-----------------------------------------------------------------------
        # Find best waypoint index
        #-----------------------------------------------------------------------
        best_traffic_index = sys.maxint

        for light in msg.lights:

            if light.state == TrafficLight.GREEN:
                continue

            p = light.pose.pose.position
            traffic_index = self.tl_map[(p.x, p.y)]

            if traffic_index > self.car_index and traffic_index < best_traffic_index:
                best_traffic_index = traffic_index

        #-----------------------------------------------------------------------
        # Publish
        #-----------------------------------------------------------------------
        if best_traffic_index == sys.maxint:
            self.best_traffic_index = None
        else:
            self.best_traffic_index = best_traffic_index
            self.time_received = rospy.get_time()

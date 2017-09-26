#-------------------------------------------------------------------------------
# Author: Mithi Sevilla <mithi.sevilla@gmail.com>
# Author: Lukasz Janyst <lukasz@jany.st>
# Date:   26.09.2017
#-------------------------------------------------------------------------------

"""
Base for a Traffic Lights Detector
"""

import rospy

from std_msgs.msg import Int32
from styx_msgs.msg import Lane

STALE_TIME = 2.0

#-------------------------------------------------------------------------------
def get_square_gap(a, b):
    """Returns squared euclidean distance between two 2D points"""
    dx = a.x - b.x
    dy = a.y - b.y
    return dx * dx + dy * dy

#-------------------------------------------------------------------------------
def get_closest_waypoint_index(light_position, waypoints):
    """
    light_position: an object with x and y positions
    waypoints: list of styx_msgs.msg.Waypoint instances
    returns index of the closest waypoint in the list waypoints
    """
    best_gap = float('inf')
    best_index = 0

    for i, waypoint in enumerate(waypoints):
        waypoint_position = waypoint.pose.pose.position
        gap = get_square_gap(light_position, waypoint_position)

        if gap < best_gap:
            best_index, best_gap = i, gap

    return best_index

#-------------------------------------------------------------------------------
class Detector(object):
    """A base class for a traffic light detector"""

    #---------------------------------------------------------------------------
    def __init__(self):
        """Initialize the detector"""

        rospy.init_node('tl_detector')
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb, queue_size=1)
        rospy.Subscriber('/car_index', Int32, self.car_index_cb, queue_size=1)

        self.publisher = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.tl_map = {}

        self.base_waypoints = None
        self.car_index = None

        self.best_traffic_index = None
        self.time_received = rospy.get_time()

    #---------------------------------------------------------------------------
    def loop(self):
        """Publish traffic light index"""

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():

            rate.sleep()

            if (rospy.get_time() - self.time_received) > STALE_TIME:
                continue

            if self.best_traffic_index is not None:
                self.publisher.publish(self.best_traffic_index)

    #---------------------------------------------------------------------------
    def car_index_cb(self, msg):
        """Car index callback"""
        self.car_index = msg.data

    #---------------------------------------------------------------------------
    def base_waypoints_cb(self, msg):
        """Base waypoints callback"""
        if self.base_waypoints is None:
            self.base_waypoints = msg.waypoints

    #---------------------------------------------------------------------------
    def fill_tl_map(self, lights):
        """Initialize traffic map dictionary"""
        for light in lights:
            traffic_index = get_closest_waypoint_index(light,
                                                       self.base_waypoints)
            self.tl_map[(light.x, light.y)] = traffic_index

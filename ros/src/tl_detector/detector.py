#-------------------------------------------------------------------------------
# Author: Mithi Sevilla <mithi.sevilla@gmail.com>
# Author: Lukasz Janyst <lukasz@jany.st>
# Date:   26.09.2017
#-------------------------------------------------------------------------------

"""
Base for a Traffic Lights Detector
"""

from collections import namedtuple
import rospy
import yaml

from std_msgs.msg import Int32
from styx_msgs.msg import Lane

Point = namedtuple('Point', ['x', 'y'])

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
    waypoints: a list of objects with x an y positions
    returns index of the closest waypoint in the list waypoints
    """
    best_gap = float('inf')
    best_index = 0

    for i, waypoint in enumerate(waypoints):
        gap = get_square_gap(light_position, waypoint)

        if gap < best_gap:
            best_index, best_gap = i, gap

    return best_index

#-------------------------------------------------------------------------------
class Detector(object):
    """A base class for a traffic light detector"""

    #---------------------------------------------------------------------------
    def __init__(self):
        """Initialize the detector"""

        #-----------------------------------------------------------------------
        # Set ROS up
        #-----------------------------------------------------------------------
        rospy.init_node('tl_detector')
        rospy.logwarn("Initializing the traffic light detector\n"
                      "The car will not move until it's initialized")

        #-----------------------------------------------------------------------
        # Read the config data
        #-----------------------------------------------------------------------
        config_string = rospy.get_param("/traffic_light_config")
        config = yaml.load(config_string)
        self.camera_info = config['camera_info']
        self.stop_line_positions = [Point(x, y) for x, y in config['stop_line_positions']]

        self.stop_map = {}
        self.base_waypoints = None
        self.classifier_initialized = False
        self.car_index = None
        self.best_stop_line_index = None
        self.time_received = rospy.get_time()

        #-----------------------------------------------------------------------
        # Set up the subscribers and the publisher
        #-----------------------------------------------------------------------
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb, queue_size=1)
        rospy.Subscriber('/car_index', Int32, self.car_index_cb, queue_size=1)
        self.publisher = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

    #---------------------------------------------------------------------------
    def is_initialized(self):
        """Check if the detector is initialized"""
        return self.base_waypoints is not None and self.classifier_initialized

    #---------------------------------------------------------------------------
    def loop(self):
        """Publish traffic light index"""

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            rate.sleep()

            if (rospy.get_time() - self.time_received) > STALE_TIME and \
               self.is_initialized():
                continue

            if self.best_stop_line_index is not None:
                self.publisher.publish(self.best_stop_line_index)

    #---------------------------------------------------------------------------
    def car_index_cb(self, msg):
        """Car index callback"""
        self.car_index = msg.data
        if self.best_stop_line_index is None and not self.is_initialized():
            self.time_received = rospy.get_time()
            self.best_stop_line_index = self.car_index + 2

    #---------------------------------------------------------------------------
    def base_waypoints_cb(self, msg):
        """Base waypoints callback"""
        if self.base_waypoints is None:
            base_waypoints = [x.pose.pose.position for x in msg.waypoints]
            for stop in self.stop_line_positions:
                stop_index = get_closest_waypoint_index(stop, base_waypoints)
                self.stop_map[stop] = stop_index
            self.base_waypoints = base_waypoints

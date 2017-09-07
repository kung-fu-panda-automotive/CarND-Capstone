#!/usr/bin/env python
""" This node will publish waypoints from the car's current position
to some `x` distance ahead with respective target speeds for each waypoint.
"""

import rospy

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane
#from styx_msgs.msg import Waypoint
#from styx_msgs.msg import TrafficLight
import waypoint_helper

MPH_TO_MPS = 0.44704
MAX_SPEED = 10.0 * MPH_TO_MPS #: Vehicle speed limit
LOOKAHEAD_WPS = 20  #: Number of waypoints we will publish
WAYPOINTS_AHEAD = 20 #: Number of waypoint traffic light is ahead of car to stop
STALE_TIME = 2.0 #: Time since that indicates it is relatively new data

class WaypointUpdater(object):
    """Given the position and map this object publishes
       points with target velocities representing path ahead"""

    def __init__(self):
        """Initialize Waypoint Updater"""

        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoints', PoseStamped, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.car_index_pub = rospy.Publisher('car_index', Int32, queue_size=1)

        self.base_waypoints = None
        self.pose = None #: Current vehicle location + orientation
        self.previous_car_index = 0 #: Where in base waypoints list the car is
        self.traffic_index = -1 #: Where in base waypoints list the traffic light is
        self.traffic_time_received = rospy.get_time() #: When traffic light info was received

        rospy.spin()

    def pose_cb(self, msg):
        """ Publishes waypoints with target velocities whenever the vehicle location is received"""

        self.pose = msg.pose # store location (x, y)
        frame_id = msg.header.frame_id

        if self.base_waypoints is not None:

            # Where in base waypoints list the car is
            car_index = waypoint_helper.get_closest_waypoint_index(self.pose, self.base_waypoints)

            # Traffic light must be in front and not too far ahead and relatively new
            cond1 = 0 <= (self.traffic_index - car_index) < WAYPOINTS_AHEAD
            cond2 = rospy.get_time() - self.traffic_time_received < STALE_TIME

            # Ask to cruise or brake car
            target_speed = MAX_SPEED
            if cond1 and cond2:
                target_speed = 0.0

            # Get subset waypoints ahead of vehicle and set target speeds
            lookahead_waypoints = waypoint_helper.get_next_waypoints(self.base_waypoints,
                                                                     car_index, LOOKAHEAD_WPS)
            for waypoint in lookahead_waypoints:
                waypoint.twist.twist.linear.x = target_speed

            # Publish
            lane = waypoint_helper.make_lane_object(frame_id, lookahead_waypoints)
            self.final_waypoints_pub.publish(lane)
            self.car_index_pub.publish(car_index)

            # Print some stuff for debugging
            if car_index != self.previous_car_index:
                self.previous_car_index = car_index
                if cond1 and cond2:
                    rospy.logwarn("WPUpdater:BRAKE!car-%s:light-%s", car_index, self.traffic_index)
                else:
                    rospy.logwarn("WPUpdater:CRUISE!car-%s:light-%s", car_index, self.traffic_index)

    def base_waypoints_cb(self, msg):
        """ We store the given map """
        # msg: styx_msgs.msg.lane
        self.base_waypoints = msg.waypoints

    def traffic_cb(self, msg):
        """ Consider traffic light when setting waypoint velocity """
        self.traffic_index = msg.data
        self.traffic_time_received = rospy.get_time()

    def obstacle_cb(self, msg):
        # pylint: disable=no-self-use
        """ Consider obstacles when setting waypoint velocity"""
        pass

    def get_waypoint_velocity(self, waypoint):
        # pylint: disable=no-self-use
        """ Get velocity of a given waypoints """
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        # pylint: disable=no-self-use
        """ Set the velocity of a given waypoint """
        waypoints[waypoint].twist.twist.linear.x = velocity


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

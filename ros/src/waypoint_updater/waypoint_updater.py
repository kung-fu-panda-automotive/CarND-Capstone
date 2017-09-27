#!/usr/bin/env python
#-------------------------------------------------------------------------------
# Author: Mithi Sevilla <mithi.sevilla@gmail.com>
# Author: Maurice Loskyll <maurice@loskyll.de>
# Author: Lukasz Janyst <lukasz@jany.st>
# Date:   15.09.2017
#-------------------------------------------------------------------------------

"""
This node publishes waypoints from the car's current position
to some distance ahead with respective target speeds for each waypoint.
"""

import rospy

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane
import waypoint_helper

LOOKAHEAD_WPS = 200 #: Number of waypoints we will publish
STALE_TIME = 1

#-------------------------------------------------------------------------------
class WaypointUpdater(object):
    """
    Given the position and map this object publishes
    points with target velocities representing path ahead
    """

    #---------------------------------------------------------------------------
    def __init__(self):
        """Initialize Waypoint Updater"""

        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.car_index_pub = rospy.Publisher('car_index', Int32, queue_size=1)

        self.base_waypoints = None
        self.pose = None #: Current vehicle location + orientation
        self.frame_id = None
        self.previous_car_index = 0 #: Where in base waypoints list the car is
        self.traffic_index = -1 #: Where in base waypoints list the traffic light is
        self.traffic_time_received = rospy.get_time() #: When traffic light info was received

        self.slowdown_coefficient = rospy.get_param("~slowdown_coefficient")
        self.stopped_distance = 0.25

        self.loop()

    #---------------------------------------------------------------------------
    def loop(self):
        """
        Publishes car index and subset of waypoints with target velocities
        """
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

            if self.base_waypoints is None or self.pose is None or self.frame_id is None:
                continue

            # Where in base waypoints list the car is
            car_index = waypoint_helper.get_closest_waypoint_index(self.pose, self.base_waypoints)

            # Get subset waypoints ahead
            lookahead_waypoints = waypoint_helper.get_next_waypoints(self.base_waypoints,
                                                                     car_index, LOOKAHEAD_WPS)

            # Traffic light must be new and near ahead
            is_new = rospy.get_time() - self.traffic_time_received < STALE_TIME
            is_near_ahead = False

            if (self.traffic_index - car_index) > 0:
                d = waypoint_helper.distance(self.base_waypoints, car_index, self.traffic_index)
                car_wp = self.base_waypoints[car_index]
                if d < car_wp.twist.twist.linear.x ** self.slowdown_coefficient:
                    is_near_ahead = True

            # Set target speeds
            if is_new and is_near_ahead:
                # Slow down and stop
                for i, waypoint in enumerate(lookahead_waypoints):
                    _, waypoint.twist.twist.linear.x = self.get_distance_speed_tuple(car_index + i)

            # Publish
            lane = waypoint_helper.make_lane_object(self.frame_id, lookahead_waypoints)
            self.final_waypoints_pub.publish(lane)
            self.car_index_pub.publish(car_index)

    #---------------------------------------------------------------------------
    def pose_cb(self, msg):
        """ Update vehicle location """
        self.pose = msg.pose # store location (x, y)
        self.frame_id = msg.header.frame_id

    #---------------------------------------------------------------------------
    def base_waypoints_cb(self, msg):
        """ Store the given map """
        self.base_waypoints = msg.waypoints

    #---------------------------------------------------------------------------
    def traffic_cb(self, msg):
        """
        Consider traffic light when setting waypoint velocity
        """
        self.traffic_index = msg.data
        self.traffic_time_received = rospy.get_time()

    #---------------------------------------------------------------------------
    def get_distance_speed_tuple(self, index):
        """
        Return tuple of distance from traffic light
        and target speed for slowing down
        """
        d = waypoint_helper.distance(self.base_waypoints, index, self.traffic_index)
        car_wp = self.base_waypoints[index]
        car_speed = car_wp.twist.twist.linear.x
        speed = 0.0

        if d > self.stopped_distance:
            speed = (d - self.stopped_distance) * (car_speed ** (1-self.slowdown_coefficient))

        if speed < 1.0:
            speed = 0.0
        return d, speed

#-------------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

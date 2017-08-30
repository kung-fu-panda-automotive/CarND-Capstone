#!/usr/bin/env python
#-------------------------------------------------------------------------------
# Author: xxx xxx <xxx@xxx.xxx>
# Date:   xx.xx.xx
#-------------------------------------------------------------------------------

"""
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
"""

import rospy

from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane
#from styx_msgs.msg import Waypoint
#from styx_msgs.msg import TrafficLight

import waypoint_helper

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
MPH_TO_MPS = 0.44704


class WaypointUpdater(object):
    """Given the position and map this object returns
       points with target velocities representing path ahead"""

    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped,
                         self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane,
                         self.base_waypoints_cb, queue_size=1)
        #rospy.Subscriber('/obstacle_waypoints', PoseStamped, self.obstacle_cb)
        #rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        # Current pose of the vehicle
        self.pose = None

        # Current list of base waypoints from Lane object
        self.base_waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
        """ Publishes waypoints with target velocities whenever the vehicle location is received"""
        rospy.loginfo('WaypointUpdater: Current Pose received.')

        # store location (x, y)
        self.pose = msg.pose

        frame_id = msg.header.frame_id

        if self.base_waypoints is not None:

            # get closest waypoint
            index = waypoint_helper.get_closest_waypoint_index(
                self.pose, self.base_waypoints)

            # make list of n waypoints ahead of vehicle
            lookahead_waypoints = waypoint_helper.get_next_waypoints(
                self.base_waypoints, index, LOOKAHEAD_WPS)

            # set velocity of all waypoints
            for waypoint in lookahead_waypoints:
                waypoint.twist.twist.linear.x = 15 * MPH_TO_MPS

            # make lane data structure to be published
            lane = waypoint_helper.make_lane_object(
                frame_id, lookahead_waypoints)

            # publish the subset of waypoints ahead
            self.final_waypoints_pub.publish(lane)

    def base_waypoints_cb(self, msg):
        """We store the given map """
        # msg: styx_msgs.msg.lane
        rospy.loginfo('WaypointUpdater: base waypoints received')
        self.base_waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # pylint: disable=no-self-use
        "Consider traffic light when setting waypoint velocity"
        rospy.loginfo('WaypointUpdater: traffic waypoint received')

    def obstacle_cb(self, msg):
        # pylint: disable=no-self-use
        "Consider obstacles when setting waypoint velocity"
        rospy.loginfo('WaypointUpdater: obstacle waypoints received')
        # Consider obstacles when setting waypoint velocity

    def get_waypoint_velocity(self, waypoint):
        # pylint: disable=no-self-use
        """Get velocity of a given waypoints"""
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        # pylint: disable=no-self-use
        """sets the velocity of a given waypoint"""
        waypoints[waypoint].twist.twist.linear.x = velocity


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

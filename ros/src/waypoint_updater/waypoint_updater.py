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

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
"""

#pylint: disable=fixme

import math

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane
#from styx_msgs.msg import Waypoint
#from styx_msgs.msg import TrafficLight

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

def distance(waypoints, wp1, wp2):
    """
    STUB
    """
    dist = 0
    dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
    for i in range(wp1, wp2+1):
        dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
        wp1 = i
    return dist

# Using code from path planning project
def closest_waypoint_(x, y, wpx, wpy, wp_len):
    """
    STUB
    """
    closestLen = 100000.0
    closestWaypoint = 0

    for i in range(wp_len):
        wp__x = wpx[i]
        wp__y = wpy[i]
        dist = math.sqrt((wp__x-x)**2+(wp__y-y)**2)
        if dist < closestLen:
            closestLen = dist
            closestWaypoint = i
    return closestWaypoint


class WaypointUpdater(object):

    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	#rospy.Subscriber('/traffic_waypoint', TrafficLight, self.traffic_cb)
	#rospy.Subscriber('/obstacle_waypoint', ??? , self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

	# Adding arrays for x-,y-coordinates, waypoint length and entire waypoint info for later use
        self.wp_x = []
        self.wp_y = []
        self.wp = []
        self.wp_len = 0

        rospy.spin()

    def pose_cb(self, msg):
	# Using the position data from the sim and the way_point's x-,y-coordinates to calculate
	# the closest waypoint to the car
        pose_x = msg.pose.position.x
        pose_y = msg.pose.position.y
        closest_waypoint = closest_waypoint_(pose_x, pose_y, self.wp_x, self.wp_y, self.wp_len)

	# Appending all waypoint information of waypoints infront of the car to waypoints_ahead
        waypoints_ahead = []
        for i in range(LOOKAHEAD_WPS):
            waypoints_ahead.append(self.wp[closest_waypoint+i])

	# Structure the waypoint data to match the desired output of Lane.msg
        lane = Lane()
        lane.waypoints = waypoints_ahead
        lane.header.stamp = rospy.Time(0)
        lane.header.frame_id = msg.header.frame_id

	# Publish the final waypoints
        self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
	# Adding the waypoints data to new_wp_x, new_wp_y, new_wp and assigning them to wp_x, wp_y, wp
        new_wp_x = []
        new_wp_y = []
        new_wp = []
        new_wp_len = len(waypoints.waypoints)
        for waypoint in waypoints.waypoints:
            new_wp_x.append(waypoint.pose.pose.position.x)
            new_wp_y.append(waypoint.pose.pose.position.y)
            new_wp.append(waypoint)

        self.wp_x = new_wp_x
        self.wp_y = new_wp_y
        self.wp = new_wp
        self.wp_len = new_wp_len

    def traffic_cb(self, msg):
        #pylint: disable=no-self-use
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        #pylint: disable=no-self-use
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        #pylint: disable=no-self-use
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        #pylint: disable=no-self-use
        waypoints[waypoint].twist.twist.linear.x = velocity


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

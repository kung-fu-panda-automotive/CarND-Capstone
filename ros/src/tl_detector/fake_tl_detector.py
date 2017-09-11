#!/usr/bin/env python

""" Fake Traffic Lights Detector to be used for testing"""
#pylint: disable=fixme

import rospy
from std_msgs.msg import Int32
from styx_msgs.msg import TrafficLightArray, TrafficLight, Lane
from copy import deepcopy

FAR_AWAY = 1000000000
STALE_TIME = 2.0

def get_square_gap(a, b):
    """Returns squared euclidean distance between two 2D points"""
    dx = a.x - b.x
    dy = a.y - b.y
    return dx * dx + dy * dy


def get_closest_waypoint_index(pose, waypoints):
    """
    pose: geometry_msg.msgs.Pose instance
    waypoints: list of styx_msgs.msg.Waypoint instances
    returns index of the closest waypoint in the list waypoints
    """
    best_gap = float('inf')
    best_index = 0
    my_position = pose.position

    for i, waypoint in enumerate(waypoints):

        other_position = waypoint.pose.pose.position
        gap = get_square_gap(my_position, other_position)

        if gap < best_gap:
            best_index, best_gap = i, gap

    return best_index


class TLDetector(object):
    """ FAKE Traffic Light Detector """

    def __init__(self):
        """ Initialize the fake detector! """

        rospy.init_node('tl_detector')
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb, queue_size=1)
        rospy.Subscriber('/car_index', Int32, self.car_index_cb, queue_size=1)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray,
                         self.traffic_cb, queue_size=1)

        self.publisher = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.traffic_map = {} #: Maps traffic light position to closest base waypoint index
        self.initialized = False # Marks that self.traffic_map is populated
        self.base_waypoints = None
        self.car_index = None
        self.best_traffic_index = None
        self.time_received = rospy.get_time()
        self.loop()

    def loop(self):
        """ Publish traffic light index """

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():

            rate.sleep()

            if (rospy.get_time() - self.time_received) > STALE_TIME:
                continue

            if self.best_traffic_index is not None:
                self.publisher.publish(self.best_traffic_index)

    def car_index_cb(self, msg):
        """ Car index callback """
        self.car_index = msg.data

    def base_waypoints_cb(self, msg):
        """ Base waypoints callback """
        if self.base_waypoints is None:
            self.base_waypoints = msg.waypoints

    def initialize(self, lights):
        """ Initialize traffic map dictionary """

        for light in lights:
            traffic_index = get_closest_waypoint_index(light.pose.pose, self.base_waypoints)
            p = light.pose.pose.position
            self.traffic_map[(p.x, p.y)] = traffic_index
        self.initialized = True

    def traffic_cb(self, msg):
        """ Determines nearest red traffic light ahead of vehicle """

        if self.base_waypoints is None or self.car_index is None:
            return

        if not self.initialized:
            self.initialize(msg.lights)

        best_traffic_index = FAR_AWAY # large number

        for light in msg.lights:

            if light.state != TrafficLight.RED:
                continue
            p = light.pose.pose.position
            traffic_index = self.traffic_map[(p.x, p.y)]

            if traffic_index > self.car_index and traffic_index < best_traffic_index:
                best_traffic_index = traffic_index
        
        if best_traffic_index == FAR_AWAY:
            self.best_traffic_index = None
        else:
            self.best_traffic_index = best_traffic_index
            self.time_received = rospy.get_time()



if __name__ == '__main__':

    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

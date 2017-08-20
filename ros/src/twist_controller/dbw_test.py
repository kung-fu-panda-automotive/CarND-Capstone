#!/usr/bin/env python
#-------------------------------------------------------------------------------
# Author: xxx xxx <xxx@xxx.xxx>
# Date:   xx.xx.xx
#-------------------------------------------------------------------------------

"""
You can use this file to test your DBW code against a bag recorded with a reference implementation.
The bag can be found in `styx/data` folder.

This file will produce 3 csv files which you can process to figure out how your DBW node is
performing on various commands.

`/actual/*` are commands from the recorded bag while `/vehicle/*` are the output of your node.

"""

import os
import csv

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd
from dbw_mkz_msgs.msg import SteeringCmd
from dbw_mkz_msgs.msg import BrakeCmd
#from dbw_mkz_msgs.msg import SteeringReport

class DBWTestNode(object):
    """
    STUB
    """
    def __init__(self):
        rospy.init_node('dbw_test_node')

        rospy.Subscriber('/vehicle/steering_cmd', SteeringCmd, self.steer_cb)
        rospy.Subscriber('/vehicle/throttle_cmd', ThrottleCmd, self.throttle_cb)
        rospy.Subscriber('/vehicle/brake_cmd', BrakeCmd, self.brake_cb)

        rospy.Subscriber('/actual/steering_cmd', SteeringCmd, self.actual_steer_cb)
        rospy.Subscriber('/actual/throttle_cmd', ThrottleCmd, self.actual_throttle_cb)
        rospy.Subscriber('/actual/brake_cmd', BrakeCmd, self.actual_brake_cb)

        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        self.steer = self.throttle = self.brake = None

        self.steer_data = []
        self.throttle_data = []
        self.brake_data = []

        self.dbw_enabled = False

        base_path = os.path.dirname(os.path.abspath(__file__))
        self.steerfile = os.path.join(base_path, 'steers.csv')
        self.throttlefile = os.path.join(base_path, 'throttles.csv')
        self.brakefile = os.path.join(base_path, 'brakes.csv')

        self.loop()

    def loop(self):
        """
        STUB
        """
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            rate.sleep()
        fieldnames = ['actual', 'proposed']

        with open(self.steerfile, 'w') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.steer_data)

        with open(self.throttlefile, 'w') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.throttle_data)

        with open(self.brakefile, 'w') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.brake_data)

    def dbw_enabled_cb(self, msg):
        """
        STUB
        """
        self.dbw_enabled = msg.data

    def steer_cb(self, msg):
        """
        STUB
        """
        self.steer = msg.steering_wheel_angle_cmd

    def throttle_cb(self, msg):
        """
        STUB
        """
        self.throttle = msg.pedal_cmd

    def brake_cb(self, msg):
        """
        STUB
        """
        self.brake = msg.pedal_cmd

    def actual_steer_cb(self, msg):
        """
        STUB
        """
        if self.dbw_enabled and self.steer is not None:
            self.steer_data.append({'actual': msg.steering_wheel_angle_cmd,
                                    'proposed': self.steer})
            self.steer = None

    def actual_throttle_cb(self, msg):
        """
        STUB
        """
        if self.dbw_enabled and self.throttle is not None:
            self.throttle_data.append({'actual': msg.pedal_cmd,
                                       'proposed': self.throttle})
            self.throttle = None

    def actual_brake_cb(self, msg):
        """
        STUB
        """
        if self.dbw_enabled and self.brake is not None:
            self.brake_data.append({'actual': msg.pedal_cmd,
                                    'proposed': self.brake})
            self.brake = None

if __name__ == '__main__':
    DBWTestNode()

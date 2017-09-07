#!/usr/bin/env python
#-------------------------------------------------------------------------------
# Author: Kostas Oreopoulos <kostas.oreopoulos@gmail.com>
# Date:   28.08.17
#-------------------------------------------------------------------------------

"""
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular
velocities. You can subscribe to any other message that you find important or refer to the document
for list of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

"""

import rospy
import dbw_helper

from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd
from dbw_mkz_msgs.msg import SteeringCmd
from dbw_mkz_msgs.msg import BrakeCmd
# from dbw_mkz_msgs.msg import SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane

from twist_controller import Controller
from yaw_controller import YawController
from speed_controller import SpeedController

PREDICTIVE_STEERING = 1.0 # from 0.0 to 1.0

class DBWNode(object):
    #pylint: disable=too-many-instance-attributes
    """
    Drive by Wire Node:
    The goal of this node is to get waypoint information and transform it
    to controller commands (throttle, brake, steer).
    The commands are published to their related channels.
    """

    def __init__(self):
        """
        1. Reads basics parameters.
        2. Subscibes to the required channels to get realtime data.
        3. Creates required channels to publish contoller actions.
        4. Creates the required controllers
        5. Starts the internal infinite loop.
        """
        rospy.init_node('dbw_node')

        # class variables
        self.dbw_enabled = True  # dbw_enabled = false => manual driving
        self.waypoints = None
        self.pose = None
        self.velocity = None
        self.twist = None

        # read ros parameters
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        # fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        # brake_deadband = rospy.get_param('~brake_deadband', .1)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        min_speed = 0.0

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        self.controller = Controller(max_steer_angle)
        self.speed_controller = SpeedController(vehicle_mass,
                                                wheel_radius,
                                                accel_limit,
                                                decel_limit)
        self.yaw_controller = YawController(wheel_base,
                                            steer_ratio,
                                            min_speed,
                                            max_lat_accel,
                                            max_steer_angle)

        # Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_cb, queue_size=1)
        rospy.Subscriber('/final_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb, queue_size=1)

        self.loop()

    def loop(self):
        """The main functionallity of the DBW Node."""
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():

            data = [self.velocity, self.waypoints, self.pose]
            all_available = all([x is not None for x in data])

            if not all_available:
                continue

            if len(self.waypoints) >= dbw_helper.POINTS_TO_FIT:

                # Read target and current velocities
                cte = dbw_helper.cte(self.pose, self.waypoints)
                target_velocity = self.waypoints[0].twist.twist.linear.x
                current_velocity = self.velocity.linear.x

                # Get corrected steering using `twist_controller`
                steer = self.controller.control(cte, self.dbw_enabled)

                # Get predicted steering angle from road curvature
                yaw_steer = self.yaw_controller.get_steering(self.twist.linear.x,
                                                             self.twist.angular.z,
                                                             current_velocity)

                throttle, brake = self.speed_controller.control(target_velocity,
                                                                current_velocity,
                                                                0.5)

                # Apply full deacceleration if target velocity is zero
                # brake = 20000 if self.twist.linear.x == 0 else brake

            else:
                # if too few waypoints and publish a hard break
                rospy.logwarn("Number of waypoint received: %s", len(self.waypoints))
                throttle, brake, steer = 0, 20000, 0

            if self.dbw_enabled:
                self.publish(throttle, brake, steer + PREDICTIVE_STEERING * yaw_steer)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        """
        Publishes the contoller commands to the corresponding topic channels

        Args:
             throttle (float): Throttle to apply
             brake (float): Brake to apply
             steer (float): Steering angle in radians
        """
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def dbw_cb(self, message):
        """From the incoming message extract the dbw_enabled variable """
        self.dbw_enabled = bool(message.data)

    def velocity_cb(self, message):
        """From the incoming message extract the velocity message """
        self.velocity = message.twist

    def pose_cb(self, message):
        """From the incoming message extract the pose message """
        self.pose = message.pose

    def twist_cb(self, message):
        """From the incoming message extract the pose message """
        self.twist = message.twist

    def waypoints_cb(self, message):
        """Update final waypoints array when a new message arrives
        on the corresponding channel
        """
        self.waypoints = message.waypoints


if __name__ == '__main__':
    DBWNode()

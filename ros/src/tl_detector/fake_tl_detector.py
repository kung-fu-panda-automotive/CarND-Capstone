#!/usr/bin/env python

""" DUMMY FOR TESTS Traffic Lights Detector """
#pylint: disable=fixme

import rospy
from std_msgs.msg import Int32

NUMBER_OF_TRAFFIC_LIGHTS = 8
COUNTER_THRESH = 200

class TLDetector(object):
    """ FAKE Traffic Lights Detector """


    def __init__(self):
        """ Initialize the detector! """
        rospy.init_node('tl_detector')
        rospy.Subscriber('/car_index', Int32, self.car_index_cb, queue_size=1)
        self.publisher = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.car_index = 0
        self.traffic_id = 0
        self.waypoint_index = 780
        self.counter = 0

        # indices of the waypoint closest to respective traffic lights
        self.traffic_lights = [780, 2047, 2580, 6294, 7008, 8540, 9733, 303]
        self.loop()


    def loop(self):
        """Publish traffic light index"""
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.publisher.publish(self.waypoint_index)
            rate.sleep()


    def car_index_cb(self, msg):
        """Car index callback"""

        car_index = msg.data

        if car_index != self.car_index:
            self.counter = 0
            self.car_index = car_index
        else:
            self.counter += 1
            if self.counter > 90:
                rospy.logwarn("TLDETECTOR: COUNT: %s", self.counter)


        if self.counter >= COUNTER_THRESH or car_index > self.waypoint_index:

            self.counter = 0
            self.traffic_id = (self.traffic_id + 1) % NUMBER_OF_TRAFFIC_LIGHTS

            while car_index > self.traffic_lights[self.traffic_id]:
                self.traffic_id += 1
                if car_index > 9733:
                    self.traffic_id = 7
                    break

            self.waypoint_index = self.traffic_lights[self.traffic_id]
            rospy.logwarn("TLDETECTOR: CAR WILL MOVE AGAIN: car_index: %s, new traffic light: %s",
                          car_index, self.waypoint_index)



if __name__ == '__main__':

    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

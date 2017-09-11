#!/usr/bin/env python
""" This is where we will write the real Traffic Lights Detector
Use fake_tl_detector for now for testing
check out tl_detector_template.py
as provided by Udacity for ideas to implement this
"""

import rospy
from fake_tl_detector import TLDetector

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

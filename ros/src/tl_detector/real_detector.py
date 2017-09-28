#-------------------------------------------------------------------------------
# Author: Lukasz Janyst <lukasz@jany.st>
# Date:   27.09.2017
#-------------------------------------------------------------------------------

"""
A Real Traffic Lights Detector
"""

import sys
import math

from detector import Detector
from classifier import Classifier
import rospy
import rospkg

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf

STATE_COUNT_THRESHOLD = 3

#-------------------------------------------------------------------------------
class RealDetector(Detector):
    """
    Traffic Lights Detector
    """
    #---------------------------------------------------------------------------
    def __init__(self):
        Detector.__init__(self)

        #-----------------------------------------------------------------------
        # Create the classifier and download the models
        #-----------------------------------------------------------------------
        self.classifier = Classifier(rospkg.get_ros_home())
        if self.classifier.need_models():
            rospy.logwarn('Need to download models, it may take a while.')
            self.classifier.download_models()
            rospy.logwarn('Finished downloading!')
        self.classifier.initialize()
        rospy.logwarn('Classifier initialized!')

        #-----------------------------------------------------------------------
        # Subscribe to the image feed and initialize other stuff
        #-----------------------------------------------------------------------
        rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)
        self.listener = tf.TransformListener()
        self.field_of_view = math.pi/4
        self.state_count = 0
        self.state = Classifier.UNKNOWN
        self.bridge = CvBridge()

    #---------------------------------------------------------------------------
    def image_cb(self, msg):
        """
        Identifies red lights in the incoming camera image and publishes the
        index of the waypoint closest to the red light to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera
        """
        if self.base_waypoints is None or self.car_index is None:
            return

        stop_wp = self.get_next_stop_wp()
        if stop_wp is None:
            self.best_stop_line_index = None
            return

        img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        state = self.classifier.classify(img)
        if self.state != state:
            self.state_count = 0
            self.state = state
            self.best_stop_line_index = None
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            if state == Classifier.RED:
                self.best_stop_line_index = stop_wp
                self.time_received = rospy.get_time()

        self.state_count += 1

    #---------------------------------------------------------------------------
    def get_next_stop_line(self):
        """
        Find index and position of the next stopping point in front of the car
        """
        next_stop_idx = sys.maxint
        next_stop = None
        for stop, stop_idx in self.stop_map.items():
            if stop_idx > self.car_index and stop_idx < next_stop_idx:
                next_stop = stop
                next_stop_idx = stop_idx

        if next_stop is None:
            return None, None
        return next_stop_idx, next_stop

    #---------------------------------------------------------------------------
    def get_next_stop_wp(self):
        """
        Get the next stop waypoint if it is close enough and in the view of
        the camera
        """

        #-----------------------------------------------------------------------
        # Check if we have the next light at all
        #-----------------------------------------------------------------------
        idx, stop = self.get_next_stop_line()
        if stop is None:
            return None

        #-----------------------------------------------------------------------
        # Convert to local coordinate frame
        #-----------------------------------------------------------------------
        stop_point = PointStamped()
        stop_point.header.stamp = rospy.get_rostime()
        stop_point.header.frame_id = '/world'
        stop_point.point.x = stop.x
        stop_point.point.y = stop.y
        stop_point.point.z = 0

        try:
            self.listener.waitForTransform("/base_link", "/world",
                                           rospy.get_rostime(),
                                           rospy.Duration(0.1))
            stop_point = self.listener.transformPoint("/base_link", stop_point)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")
            return None

        x = stop_point.point.x
        y = stop_point.point.y

        #-----------------------------------------------------------------------
        # Check if it's not too far and in the view of the camera
        #-----------------------------------------------------------------------
        if math.sqrt(x*x+y*y) > 50:
            return None

        angle = abs(math.atan2(y, x))
        if angle > self.field_of_view:
            return None

        return idx

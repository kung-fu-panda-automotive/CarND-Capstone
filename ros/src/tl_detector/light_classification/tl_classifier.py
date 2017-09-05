# -------------------------------------------------------------------------------
# Author: Alexey Simonov <alexey.simonov@gmail.com>
# Date:   05.09.17
# -------------------------------------------------------------------------------

"""
Traffic Light Classifier for Carla

This is the class in the ROS project framework provided by Udacity.

Actual implementation is delegated to TLClassifierCNN, a convolutional net classifier implemented
with tensorflow.

The development/reseach/training repo for TLClassifierCNN is
https://github.com/asimonov/Bosch-TL-Dataset. Check it out to train and update model.

TODO:
  * TLClassifierCNN opens tensorflow session. How shall we size memory for that?
    Shall we place it on CPU? It should not make much difference for inference on single-image
    batches but may avoid runtime issues.
"""

import numpy as np
import cv2

from styx_msgs.msg import TrafficLight

from .tl_classifier_cnn import TLLabelConverter, TLClassifierCNN


class TLClassifier(object):
    """
    Traffic Lights classifier

    Creates an instance of TLClassifierCNN, which creates a tensorflow session.
    Loads pre-trained model.

    Attributes:
      _label_converter: TLLabelConverter to convert string labels to other representations
      _cnn_classifier: TLClassifierCNN for actual classification
      _classifier_image_size: tuple with the image size the TLClassifierCNN expects
    """

    def __init__(self):
        self._label_converter = TLLabelConverter()
        self._classifier_image_size = (32, 32)
        self._cnn_classifier = TLClassifierCNN()
        # model path below is relative to tl_detector/ node folder
        # (see cwd="node" in tl_detector.launch )
        self._cnn_classifier.load_model('light_classification/model')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Resizes image and uses TLClassifierCNN to determine the light.

        Args:
            image (cv::Mat): image containing the traffic light. Can be arbitrary size, but expected
                             to have BGR channels, usual OpenCV format.

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        resized = cv2.resize(image, self._classifier_image_size, interpolation=cv2.INTER_LINEAR)
        assert resized.shape == (32, 32, 3)
        labels, _ = self._cnn_classifier.predict(np.array([resized]), batch_size=1)

        result = TrafficLight.UNKNOWN
        if labels[0] == 'red':
            result = TrafficLight.RED
        elif labels[0] == 'green':
            result = TrafficLight.GREEN
        elif labels[0] == 'yellow':
            result = TrafficLight.YELLOW

        return result

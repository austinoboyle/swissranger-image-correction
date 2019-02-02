#!/usr/bin/env python

from __future__ import print_function

import roslib
roslib.load_manifest('repub')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge, CvBridgeError

TS_THRESHOLD = 0.001  # Appropriate time interval between 'equal' image pairs
QUEUE_SIZE = 10  # Max images to queue in message_filter buffer


class IntensityCorrector:
    def __init__(self):
        """
        Correct SwissRanger ToF images for intensity

        Initialize publishers, subscribers, CV Bridge, and the synchronizer
        """

        # Bridge to convert from openCV -> Ros Image and vice-versa
        self.bridge = CvBridge()

        # Create publishers and subscribers
        self.image_pub = rospy.Publisher("corrected_image", Image)
        self.distance_sub = message_filters.Subscriber(
            "/SwissRanger/distance/image_raw", Image)
        self.intensity_sub = message_filters.Subscriber(
            '/SwissRanger/intensity/image_raw', Image)

        # Send pairs of images to callback when their timestamps are close to equal
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.distance_sub, self.intensity_sub], queue_size=QUEUE_SIZE, slop=TS_THRESHOLD)
        self.ts.registerCallback(self.sync_callback)

    def sync_callback(self, distance, intensity):
        """
        Publish an image topic with distance * intensity^2

        args:
            distance: cv2 image matrix from SwissRanger/distance/image_raw
            intensity: cv2 image matrix from SwissRanger/intensity/image_raw

        returns: None
        """
        try:
            d = self.bridge.imgmsg_to_cv2(distance, 'bgr8')
            i = self.bridge.imgmsg_to_cv2(intensity, 'bgr8')
            product = cv2.multiply(d, cv2.multiply(i, i))
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(product, 'bgr8'))
        except CvBridgeError as e:
            print("ERROR", e)


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = IntensityCorrector()
    while not rospy.is_shutdown():
        rospy.spin()
    print("Shutting Down")


if __name__ == '__main__':
    main(sys.argv)

#!/usr/bin/env python3
"""
Republishes an image topic with encoding normalised for cv_bridge.

RealSense SDK IR frames are stored as 8UC1 (OpenCV raw type); cv_bridge
requires mono8 for single-channel greyscale.  Color frames (rgb8) are
passed through unchanged — cv_bridge converts them to greyscale internally.
"""

import rospy
from sensor_msgs.msg import Image


class ImageEncodingFix:

    def __init__(self):
        in_topic  = rospy.get_param("~in_topic",
                                    "/device_0/sensor_0/Infrared_1/image/data")
        out_topic = rospy.get_param("~out_topic",
                                    "/camera/infra1/image_raw")

        self._pub = rospy.Publisher(out_topic, Image, queue_size=10)
        rospy.Subscriber(in_topic, Image, self._cb, queue_size=10)

        rospy.loginfo("[image_encoding_fix] %s  ->  %s", in_topic, out_topic)

    def _cb(self, msg):
        if msg.encoding == "8UC1":
            msg.encoding = "mono8"
        self._pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("image_encoding_fix")
    ImageEncodingFix()
    rospy.spin()

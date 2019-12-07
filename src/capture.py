#!/usr/bin/env python

import numpy 
import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image, PointCloud2
from stereo_msgs.msg import DisparityImage


def cb_rgb(image_message):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
    cv2.imwrite("test_rpg.jpg", cv_image)


def cb_depth(image_message):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_message.image, desired_encoding="passthrough")
    array = numpy.asarray(cv_image, dtype=numpy.uint8)
    cv2.imwrite("test_depth.jpg", cv_image)


def capture():
    rospy.init_node("capture_kinect")

    rospy.Subscriber("/camera/rgb/image_color", Image, cb_rgb)
    rospy.Subscriber("/camera/depth_registered/disparity", DisparityImage, cb_depth)

    rospy.spin()


if __name__ == "__main__":
    capture()

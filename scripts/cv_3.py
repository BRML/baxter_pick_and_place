#!/usr/bin/env python
"""
cf. Example 12-2 from Quigley, Gerkey, Smart: Programming Robots with ROS.

Program subscribes to left hand camera of Baxter and streams the images in a
OpenCV image window.

As it turns out, the 'cv2.namedWindow()' is not required, but the
'rospy.spin()' is.
"""


import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image",
                                          Image, self.image_cb)

    def image_cb(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("window", image)
        cv2.waitKey(3)


rospy.init_node('follower')
follower = Follower()
rospy.spin()

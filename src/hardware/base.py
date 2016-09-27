# Copyright (c) 2016, BRML
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import numpy as np

import cv_bridge

import rospy
from sensor_msgs.msg import Image


class Camera(object):
    def __init__(self, topic):
        """Base class for a ROS camera.
        A camera should have at least
          - a method to read images from a ROS topic,
          - a method to project pixel coordinates to camera coordinates, and
          - a method to project camera coordinates to pixel coordinates.
        """
        self._topic = topic
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        # use camera_info to get fx, fy, cx, cy?
        self._camera_matrix = np.array([self.fx, 0.0, self.cx, 0.0,
                                        0.0, self.fy, self.cy, 0.0,
                                        0.0, 0.0, 1.0, 0.0,
                                        0.0, 0.0, 1.0, 0.0]).reshape((4, 4))

    def collect_image(self):
        """Get the most recent image message from the ROS topic and convert it
        into a numpy array.

        :return: A numpy array holding the image.
        """
        # http://docs.ros.org/api/rospy/html/rospy.client-module.html#wait_for_message
        # imgmsg = rospy.wait_for_message(topic=self._topic, topic_type=Image,
        #                                 timeout=0.5)
        # img = imgmsg2img(imgmsg=imgmsg)
        # print img.shape
        # return img
        return np.zeros((1280, 800, 3))

    def projection_pixel_to_camera(self, pixel):
        raise NotImplementedError()

    def projection_camera_to_pixel(self, position):
        raise NotImplementedError()


def imgmsg_to_img(imgmsg):
    """ Convert a ROS image message to a numpy array holding the image.
    :param imgmsg: a ROS image message
    :return: a numpy array containing an RGB image
    """
    try:
        img = cv_bridge.CvBridge().imgmsg_to_cv2(imgmsg, 'bgr8')
    except cv_bridge.CvBridgeError as e:
        raise e
    except AttributeError as e:
        raise e
    return img

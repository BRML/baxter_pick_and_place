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
import os

import cv2
import cv_bridge

import rospy
from sensor_msgs.msg import (
    CameraInfo,
    Image
)


class Camera(object):
    def __init__(self, topic):
        """Base class for a ROS camera.
        A camera should have at least
          - a method to read images from a ROS topic,
          - a method to project pixel coordinates to camera coordinates, and
          - a method to project camera coordinates to pixel coordinates.
        """
        self._topic = topic

        self._camera_matrix = self._get_calibration()

    def _get_calibration(self):
        """Read the calibration data of the camera from either a ROS topic or
        a calibration file. For additional information see
        http://docs.ros.org/indigo/api/sensor_msgs/html/msg/CameraInfo.html.

        :return: The projection/camera matrix (a 3x4 numpy array).
        """
        topic = self._topic.rsplit('/', 1)[0] + '/camera_info'
        try:
            # try to read calibration from ROS camera info topic
            msg = rospy.wait_for_message(topic=topic, topic_type=CameraInfo,
                                         timeout=0.5)
            cal = msg.P
        except rospy.ROSException:
            # fall back to stored calibration values
            # TODO: load calibration from file
            cal = [2, 0, 1, 0, 0, 2, 1, 0, 0, 0, 1, 0]
        return np.asarray(cal).reshape((3, 4))

    def collect_image(self):
        """Read the most recent image message from the ROS topic and convert
        it into a numpy array.

        :return: An image (a (height, width, n_channels) numpy array).
        """
        try:
            msg = rospy.wait_for_message(topic=self._topic,
                                         topic_type=Image,
                                         timeout=0.5)
            img = imgmsg_to_img(imgmsg=msg)
        except rospy.ROSException:
            # TODO: replace this debugging stuff with 'img = None'
            path = '/home/mludersdorfer/software/ws_baxter_pnp/src/baxter_pick_and_place'
            img_files = ['004545', '000456', '000542', '001150', '001763',
                         '2008_000533', '2008_000910', '2008_001602',
                         '2008_001717', '2008_008093']
            idx = np.random.randint(len(img_files))
            img_file = os.path.join(path, 'data', '%s.jpg' % img_files[idx])
            img = cv2.imread(img_file)
        return img

    def projection_pixel_to_camera(self, pixel):
        # TODO: implement
        pass

    def projection_camera_to_pixel(self, position):
        """Project a 3d point [x, y, z] in camera coordinates onto the
        rectified image. For additional information see
        http://docs.ros.org/indigo/api/sensor_msgs/html/msg/CameraInfo.html.

        :param position: A 3D position as a list of length 3 [x, y, z].
        :return: The corresponding pixel coordinates (px, py).
        """
        if isinstance(position, list) and len(position) == 3:
            hom = np.asarray(position + [1])
            u, v, w = np.dot(self._camera_matrix, hom)
            px = float(u/w)
            py = float(v/w)
            return px, py
        raise ValueError("'position' should be a list of length 3!")


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

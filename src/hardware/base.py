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

        self.meters_per_pixel = None

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

    def projection_pixel_to_camera(self, pixel, z):
        """Project a 2d point (px, py) in pixel coordinates into 3D camera
        coordinates.
        Since the projection from camera to pixel space is non-invertible,
        we are restricted to the case that we know the z coordinate of the
        point in camera coordinates, e.g., for an object lying on a table
        with known height.

        :param pixel: A 2D position as a tuple of length 2 (px, py).
        :param z: The known height of the pixel in camera coordinates.
        :return: The corresponding 3D camera coordinates [x, y, z].
        """
        if isinstance(pixel, (tuple, list)) and len(pixel) == 2:
            # For known z coordinate we can rearrange
            #   (u)   [fx 0  cx](x)
            #   (v) = [0  fy cy](y)
            #   (w)   [0  0  1 ](z)
            # to compute
            #   w = z,
            #   x = (u - cx*z)/fx,  u = px*w, and
            #   y = (v - cy*z)/fy,  v = py*w.
            u, v = [p*z for p in pixel]
            x = (u - self._camera_matrix[0, 2]*z)/self._camera_matrix[0, 0]
            y = (v - self._camera_matrix[1, 2]*z)/self._camera_matrix[1, 1]
            return [x, y, z]
        raise ValueError("'pixel' should be a tuple of length 2!")

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
    """Convert a ROS image message to a numpy array holding the image.

    :param imgmsg: A ROS image message.
    :return: The BGR image as a (height, width, n_channels) numpy array.
    """
    img = None
    for enc in ['bgr8', 'mono8', '32FC1']:
        try:
            img = cv_bridge.CvBridge().imgmsg_to_cv2(imgmsg, enc)
        except cv_bridge.CvBridgeError:
            pass
        except AttributeError as e:
            raise e
        if img is not None:
            break
    if img is None:
        raise ValueError("Cannot convert image message to numpy array!")
    return img


def img_to_imgmsg(img):
    """Convert a numpy array holding an image to a ROS image message.

    :param img: A BGR image as a (height, width, n_channels) numpy array.
    :return: The corresponding ROS image message.
    """
    imgmsg = None
    for enc in ['bgr8', 'mono8', '32FC1']:
        try:
            imgmsg = cv_bridge.CvBridge().cv2_to_imgmsg(img, enc)
        except cv_bridge.CvBridgeError:
            pass
        if imgmsg is not None:
            break
    if imgmsg is None:
        raise ValueError("Cannot convert {} {} array to image message!".format(
            img.shape, img.dtype))
    return imgmsg

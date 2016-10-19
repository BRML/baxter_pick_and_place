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

import logging
import numpy as np

import cv_bridge

import rospy
from sensor_msgs.msg import (
    CameraInfo,
    Image
)


# Set up logging
_logger = logging.getLogger('cam')
_logger.setLevel(logging.DEBUG)
_default_loghandler = logging.StreamHandler()
_default_loghandler.setLevel(logging.DEBUG)
_default_loghandler.setFormatter(logging.Formatter('[%(name)s][%(levelname)s] %(message)s'))
_logger.addHandler(_default_loghandler)


def remove_default_loghandler():
    """Call this to mute this library or to prevent duplicate messages
    when adding another log handler to the logger named 'cam'."""
    _logger.removeHandler(_default_loghandler)


class Camera(object):
    def __init__(self, topic, cam_pars=None):
        """Base class for a ROS camera.
        A camera should have at least
          - a method to read images from a ROS topic,
          - a method to project pixel coordinates to camera coordinates, and
          - a method to project camera coordinates to pixel coordinates.

        :param topic: The ROS image topic to read camera images from.
        :param cam_pars: An optional dictionary containing camera parameters.
            If given it replaces the parameters read from the camera driver
            via ROS and is required to contain
                - a 3x3 camera matrix,
                - the image size (height, width) and
                - the camera distortion coefficients.
        """
        self._topic = topic

        self.camera_matrix = None
        self.image_size = None
        self.distortion_coeff = None
        if cam_pars is None:
            self._get_ros_calibration()
        else:
            self.camera_matrix = cam_pars['cam_mat']
            self.image_size = tuple(cam_pars['size'])
            self.distortion_coeff = cam_pars['dist_coeff']
        if self.camera_matrix.shape != (3, 3):
            raise ValueError("Expected a 3x3 camera matrix, got "
                             "{}!".format(self.camera_matrix.shape))

        self.meters_per_pixel = None

    def _get_ros_calibration(self):
        """Read the calibration data of the camera from the ROS topic. For
        additional information see
        http://docs.ros.org/indigo/api/sensor_msgs/html/msg/CameraInfo.html.

        :return:
        """
        topic = self._topic.rsplit('/', 1)[0] + '/camera_info'
        _logger.info("Try to read camera info from {}.".format(topic))
        try:
            # try to read calibration from ROS camera info topic
            msg = rospy.wait_for_message(topic=topic, topic_type=CameraInfo,
                                         timeout=1.5)
            self.camera_matrix = np.asarray(msg.K, dtype=np.float64).reshape((3, 3))
            self.image_size = (int(msg.height), int(msg.width))
            self.distortion_coeff = np.asarray(msg.D, dtype=np.float64)
        except rospy.ROSException:
            raise RuntimeError("Unable to read camera info from ROS master!")

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
            msg = "ROS error while reading image from {}.".format(self._topic)
            _logger.error(msg)
            raise RuntimeError(msg)
        if img.dtype == np.float32:
            # In simulation, depth map is a float32 image
            mask = np.isnan(img)
            if mask.any():
                # In simulation, the background has NaN depth values.
                # We replace them with 0 m, similar to what the Kinect V1 did.
                # See https://msdn.microsoft.com/en-us/library/jj131028.aspx.
                _logger.debug("{}: There was at least one NaN in the depth "
                              "image. I replaced all occurrences with "
                              "0.0 m.".format(self._topic.rsplit('/', 1)[0]))
                img.flags.writeable = True
                img[mask] = 0.0
                # We now map the float values in meters to uint16 values in mm
                # as provided by the libfreenect2 library and Kinect SDK.
                img *= 1000.0
                img = img.astype(np.uint16, copy=False)
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
            x = (u - self.camera_matrix[0, 2] * z) / self.camera_matrix[0, 0]
            y = (v - self.camera_matrix[1, 2] * z) / self.camera_matrix[1, 1]
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
            u, v, w = np.dot(self.camera_matrix, np.asarray(position))
            px = float(u)/w
            py = float(v)/w
            return px, py
        raise ValueError("'position' should be a list of length 3!")


def imgmsg_to_img(imgmsg):
    """Convert a ROS image message to a numpy array holding the image.

    :param imgmsg: A ROS image message.
    :return: The BGR image as a (height, width, n_channels) numpy array.
    """
    img = None
    for enc in ['bgr8', 'mono8', 'passthrough']:
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
    for enc in ['bgr8', 'mono8', 'passthrough']:
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

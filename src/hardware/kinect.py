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

# DISCLAIMER: The client interface to the ELTE Kinect Windows tool is adapted
# from and inspired by software written by Mike Olasz at ELTE.

import logging
import numpy as np
from numpy.random import random_sample
import os
import socket
import struct
import time

import cv2

import rospy
from sensor_msgs.msg import CameraInfo

from base import Camera
from demo.settings import task_space_limits_m as lims


# Set up logging
_logger = logging.getLogger('kinect')
_logger.setLevel(logging.INFO)
_default_loghandler = logging.StreamHandler()
_default_loghandler.setLevel(logging.INFO)
_default_loghandler.setFormatter(logging.Formatter('[%(name)s][%(levelname)s] %(message)s'))
_logger.addHandler(_default_loghandler)


def remove_default_loghandler():
    """Call this to mute this library or to prevent duplicate messages
    when adding another log handler to the logger named 'kinect'."""
    _logger.removeHandler(_default_loghandler)


class Kinect(object):
    def __init__(self, root_dir, host=None):
        cm_color = None
        cm_depth = None
        self._host = host
        self._socket = None
        self._native_ros = False
        try:
            # try to read calibration from ROS camera info topic
            _ = rospy.wait_for_message(topic='/kinect2/sd/camera_info',
                                       topic_type=CameraInfo,
                                       timeout=0.5)
            self._native_ros = True
        except rospy.ROSException:
            # Load previously stored camera matrices for color (1920x1080)
            # and depth (512x424) sensors. The values in kinect_params.npz
            # are obtained by running
            # $ roslaunch kinect2_bridge kinect2_bridge.launch
            # $ rostopic echo -n 1 /kinect2/sd/camera_info (depth)
            # $ rostopic echo -n 1 /kinect2/hd/camera_info (color with 1920x1080)
            # $ rostopic echo -n 1 /kinect2/qhd/camera_info (color with 960x540)
            # Note that qhd is scaled from hd by a factor of 1/2 in image size
            # and in fx, fy, cx, cy. It therefore is sufficient to save the hd
            # values.
            path = os.path.join(root_dir, 'data', 'setup', 'kinect_params.npz')
            with np.load(path) as cal:
                cm_color = cal['hd']
                # ELTE KinectOverNetwork tool scales the color image by a
                # factor of 1/2 (960x540).
                cm_color /= 2.0
                cm_depth = cal['depth']
        self.depth = Camera(topic='/kinect2/sd/image_depth_rect', cam_mat=cm_depth)
        self.color = Camera(topic='/kinect2/hd/image_color_rect', cam_mat=cm_color)

        # index into the skeleton arrays
        self.joint_type_hand_left = 7
        self.joint_type_hand_right = 11

    def _receive_data(self):
        """Receive a given number of bytes from the data stream provided by
        the ELTE Kinect Windows tool.

        :return: The received data.
        :raise ValueError: If the requested number of bytes was not received.
        """
        start = time.time()
        # Reading the size of the data stream we want to read
        data = self._socket.recv(4)
        n_bytes = struct.unpack("<I", data)[0]
        _logger.debug("Need to receive {} bytes.".format(n_bytes))
        # Sending ACK that we received the size
        self._socket.sendall("OK\n")
        # Reading the socket stream to get every packet
        data = self._socket.recv(n_bytes)
        while len(data) < n_bytes:
            data += self._socket.recv(n_bytes - len(data))
        _logger.info("Received {}/{} bytes in {:.3f} s.".format(len(data),
                                                                n_bytes, time.time() - start)
                     )
        if len(data) != n_bytes:
            msg = "Received {} but should have received {} bytes!".format(
                len(data), n_bytes)
            _logger.error(msg)
            raise ValueError(msg)
        # Sending ACK that we received the data
        self._socket.sendall("OK2\n")
        return data

    def _receive_color(self):
        """Receive a color image from the ELTE Kinect Windows tool and decode
        it as a uint8, three-channel numpy array of size height x width.

        :return: A (h, w, 3) numpy array holding the color image.
        """
        msg = self._receive_data()
        barray = np.fromstring(msg, np.uint8)
        img = cv2.imdecode(barray, cv2.IMREAD_COLOR)
        _logger.debug("Received a {} {} color image (min={}, max={}).".format(
            img.shape, img.dtype, img.min(), img.max())
        )
        return img

    def _receive_depth(self):
        """Receive a depth image from the ELTE Kinect Windows tool and decode
        it as an uint16 depth map where each value represents the distance in
        millimeters. The maximum depth distance is 8 meters, although
        reliability starts to degrade at around 4.5 meters.
        See https://msdn.microsoft.com/en-us/library/windowspreview.kinect.depthframe.aspx
        for more information.

        :return: A (h, w) numpy array holding the depth map.
        """
        msg = self._receive_data()
        barray = np.fromstring(msg, np.uint16)
        img = cv2.imdecode(barray, cv2.IMREAD_UNCHANGED)
        _logger.debug("Received a {} {} depth image (min={}, max={}).".format(
            img.shape, img.dtype, img.min(), img.max())
        )
        return img

    def _receive_skeleton_data(self):
        """Receive skeleton data from the ELTE Kinect Windows tool.

        :return: A tuple containing the number of bodies (skeletons) in the
            data and the received data.
        """
        # Reading the number of bodies we want to read
        data = self._socket.recv(4)
        if len(data) != 4:
            # Kinect did not find a skeleton to track
            return 0, None
        n_bodies = struct.unpack("<I", data)[0]
        # Sending ACK that we received the size
        self._socket.sendall("OK\n")
        data = self._receive_data()
        return n_bodies, data

    def _receive_skeleton(self):
        """Receive skeleton data from the ELTE Kinect Windows tool and decode
        if as a list of length number of bodies (skeletons), where each
        skeleton is defined by three lists holding 13 joint coordinates each,
        where the first list holds the camera coordinates, the second list
        holds color space (pixel) coordinates and the third list holds depth
        space (pixel) coordinates.

        :return: A list holding n skeletons defined by three lists of joint
            coordinates in camera, color and depth space.
        """
        n_bodies, msg = self._receive_skeleton_data()
        if n_bodies == 0:
            return None
        barray = np.fromstring(msg, np.float32)
        bodies = list()
        # TODO: does this work for n_bodies > 1?
        for _ in range(n_bodies):
            cam_space_points = list()
            color_space_points = list()
            depth_space_points = list()
            for i in range(0, barray.size, 7):
                cam_space_points.append((barray[i], barray[i + 1], barray[i + 2]))
                color_space_points.append((barray[i + 3], barray[i + 4]))
                depth_space_points.append((barray[i + 5], barray[i + 6]))
            bodies.append((cam_space_points, color_space_points, depth_space_points))
        _logger.debug("Received skeleton data for {} bod{}.".format(
            len(bodies), 'y' if len(bodies) == 1 else 'ies')
        )
        return bodies

    def collect_data(self, color=False, depth=False, skeleton=False):
        """Collect the latest color, depth and skeleton data from the Kinect V2
        sensor.
        Note: If the Kinect is connected to a Ubuntu machine, use the native
        ROS interface using the libfreenect2 and iai_kinect2 libraries. If the
        Kinect is connected to a Windows machine and runs the ELTE Kinect
        Windows tool server, communicate via a TCP/IP socket connection.

        :param color: Whether to retrieve the latest color image.
        :param depth: Whether to retrieve the latest depth image.
        :param skeleton: Whether to retrieve the latest skeleton data.
        :return: A triple (color image, depth image, skeletons).
        """
        img_color = None
        img_depth = None
        data_skeleton = None
        if self._native_ros:
            # Kinect is connected to the Ubuntu machine, communicate via ROS
            if color:
                img_color = self.color.collect_image()
            if depth:
                img_depth = self.depth.collect_image()
            if skeleton:
                data_skeleton = None
        else:
            # Kinect is connected to a Windows machine, communicate via
            # TCP/IP socket connection
            if not self._host:
                msg = "No host name for ELTE Kinect Windows tool provided!"
                _logger.error(msg)
                raise ValueError(msg)
            # Create a TCP/IP socket
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Connect the socket to the host and port where the server listens
            server = self._host, 9999
            _logger.info('Connect to {} on port {}.'.format(server[0].upper(),
                                                            server[1]))
            self._socket.connect(server)
            try:
                msg = '{}{}{}\n'.format(*[1 if x else 0
                                          for x in [color, depth, skeleton]])
                self._socket.sendall(msg)
                if color:
                    img_color = self._receive_color()
                if depth:
                    img_depth = self._receive_depth()
                if skeleton:
                    data_skeleton = self._receive_skeleton()
            except ValueError:
                err = 'Problem when receiving the data!'
                _logger.warning(err)
                raise ValueError(err)
            finally:
                _logger.info('Close socket.')
                self._socket.close()
            self._socket = None
        return img_color, img_depth, data_skeleton

    def estimate_hand_position(self):
        """Extract the estimate for the approximate hand position from the
        skeleton data obtained from the Kinect.

        :return: One of
            - A dictionary holding the hand coordinate triplets (camera,
              color and depth space coordinates) for the left and right hands.
            - None if no skeleton estimate is computed by the Kinect.
        """
        _, _, skeletons = self.collect_data(skeleton=True)
        if len(skeletons) != 1:
            raise ValueError("Need to track exactly one person!")
        skeleton = skeletons[0]
        if skeleton is None:
            estimate = None
        else:
            estimate = dict()
            estimate['left'] = [a[self.joint_type_hand_left] for a in skeleton]
            estimate['right'] = [a[self.joint_type_hand_right] for a in skeleton]
        return estimate

    def estimate_object_position(self, img_rgb, bbox, img_depth):
        """Estimate the approximate position of an object in 3d from a Kinect
        color and corresponding depth image, as well as the bounding box of
        the object detected in the color image.

        :param img_rgb: A color image.
        :param bbox: The bounding box of the object we are interested in in
            the color image.
        :param img_depth: A depth image corresponding to the color image.
        :return: A list [x, y, z] representing the approximate hand position
            in camera coordinates.
        """
        # if bbox is None:
        #     return None
        # TODO: implement. But how?
        # requires re-projecting the center of the bounding box to 3d and
        # calibrating the color and depth images to find the proper depth value
        if random_sample() > 0.8:
            return [
                (lims['x_max'] - lims['x_min'])*random_sample() + lims['x_min'],
                (lims['y_max'] - lims['y_min'])*random_sample() + lims['y_min'],
                0.2*random_sample() - 0.1 + lims['z_min']
            ]
        else:
            return None

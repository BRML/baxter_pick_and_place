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
import os
import socket
import struct
import time

import cv2

import rospy
from sensor_msgs.msg import CameraInfo, Image

from base import Camera, img_to_imgmsg
from depth_registration import get_depth
from settings.debug import topic_img4


class Kinect(object):
    def __init__(self, root_dir, host=None):
        name = 'main.kinect'
        self._logger = logging.getLogger(name)

        pars_color = None
        pars_depth = None
        path = os.path.join(root_dir, 'data', 'setup', 'kinect_parameters.npz')
        self._pub_vis = rospy.Publisher(topic_img4, Image,
                                        queue_size=10, latch=True)
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
            self._logger.info("Loading previously saved camera info for color "
                              "and depth sensors.")
            # Load previously stored camera matrices for color (1920x1080)
            # and depth (512x424) sensors. The values in kinect_params.npz
            # are obtained using the 'calibrate_kinect.py' script.
            with np.load(path) as cal:
                # The ELTE KinectOverNetwork tool scales the color image by a
                # factor of 1/2 (to 960x540). We thus adapt the loaded camera
                # matrix accordingly.
                pars_color = {
                    'cam_mat': cal['cam_mat_color']/2.0,
                    'size': cal['size_color']//2,
                    'dist_coeff': cal['dist_coeff_color']
                }
                pars_color['cam_mat'][2, 2] *= 2.0
                pars_depth = {
                    'cam_mat': cal['cam_mat_depth'],
                    'size': cal['size_depth'],
                    'dist_coeff': cal['dist_coeff_depth']
                }
        self.depth = Camera(topic='/kinect2/sd/image_depth_rect',
                            prefix=name, cam_pars=pars_depth)
        self.color = Camera(topic='/kinect2/hd/image_color_rect',
                            prefix=name, cam_pars=pars_color)

        # index into the skeleton arrays
        self.joint_type_count = 13
        self.joint_type_hand_left = 7
        self.joint_type_hand_right = 11

        # affine transformation from camera to Baxter coordinates
        self.trafo = None

    def _receive_size(self):
        """Receive the number of bytes needed to read from the data stream.

        :return: The _image_size of the data to read (as an int).
        """
        data = self._socket.recv(4)
        try:
            size = struct.unpack('<i', data)[0]  # we receive an int value
        except ValueError:
            size = -1
        finally:
            # Sending ACK that we received the _image_size
            self._socket.sendall("OK\n")
        return size

    def _receive_data(self, n_bytes):
        """Receive a given number of bytes from the data stream provided by
        the ELTE Kinect Windows tool.

        :param n_bytes: The number of bytes to read from the data stream.
        :return: The received data.
        :raise ValueError: If the requested number of bytes was not received.
        """
        # Reading the socket stream to get every packet
        data = self._socket.recv(n_bytes)
        while len(data) < n_bytes:
            data += self._socket.recv(n_bytes - len(data))
        self._logger.debug("Received {}/{} bytes.".format(len(data), n_bytes))
        if len(data) != n_bytes:
            msg = "Received {} but should have received {} bytes!".format(
                len(data), n_bytes)
            self._logger.error(msg)
            raise ValueError(msg)
        # Sending ACK that we received the data
        self._socket.sendall("OK2\n")
        return data

    def _receive_color(self):
        """Receive a color image from the ELTE Kinect Windows tool and decode
        it as a uint8, three-channel numpy array of _image_size height x width.

        :return: A (h, w, 3) numpy array holding the color image.
        """
        start = time.time()
        # Reading the _image_size of the data stream we want to read
        n_bytes = self._receive_size()
        if n_bytes == -1:
            self._logger.warning("Failed to receive color image data!")
            return None
        self._logger.debug("Need to receive {} bytes.".format(n_bytes))

        # Reading the data from the socket stream
        try:
            msg = self._receive_data(n_bytes=n_bytes)
        except ValueError:
            return None

        # Convert received data into a numpy array
        try:
            barray = np.fromstring(msg, np.uint8)
        except ValueError:
            self._logger.warning("Error when converting raw data to color image!")
            return None
        img = cv2.imdecode(barray, cv2.IMREAD_COLOR)
        self._logger.info("Received a {} {} color image (min={}, max={}) "
                          "in {:.3f} s.".format(img.shape, img.dtype,
                                                img.min(), img.max(),
                                                time.time() - start))
        return img

    def _receive_depth(self):
        """Receive a depth image from the ELTE Kinect Windows tool and decode
        it as an uint16 depth map where each value represents the distance in
        millimeters. The maximum depth distance is 8 meters, although
        reliability starts to degrade at around 4.5 meters.
        See https://msdn.microsoft.com/en-us/library/windowspreview.kinect.depthframe.aspx
        for more information.

        :return: A (h, w) numpy array holding the depth map.
        :raise ValueError: If no depth map was received.
        """
        start = time.time()
        # Reading the _image_size of the data stream we want to read
        n_bytes = self._receive_size()
        if n_bytes == -1:
            self._logger.warning("Failed to receive depth map data!")
            return None
        self._logger.debug("Need to receive {} bytes.".format(n_bytes))

        # Reading the data from the socket stream
        try:
            msg = self._receive_data(n_bytes=n_bytes)
        except ValueError:
            return None

        # Convert received data into a numpy array
        try:
            barray = np.fromstring(msg, np.uint16)
        except ValueError:
            self._logger.warning("Error when converting raw data to depth map!")
            return None
        img = cv2.imdecode(barray, cv2.IMREAD_UNCHANGED)
        self._logger.info("Received a {} {} depth map (min={}, max={}) "
                          "in {:.3f} s.".format(img.shape, img.dtype,
                                                img.min(), img.max(),
                                                time.time() - start))
        return img

    def _receive_skeleton_data(self, n_bytes):
        """Receive skeleton data from the ELTE Kinect Windows tool.

        :param n_bytes: The number of bytes to read from the data stream.
        :return: A tuple containing the number of bodies (skeletons) in the
            data and the received data.
        """
        # Reading the number of bodies we want to receive data for
        data = self._socket.recv(4)
        try:
            n_bodies = struct.unpack("<I", data)[0]  # read unsigned int value
        except ValueError:
            n_bodies = 0
        finally:
            # Sending ACK that we received the _image_size
            self._socket.sendall("OK\n")

        # Reading the data from the socket stream
        try:
            msg = self._receive_data(n_bytes=n_bytes)
        except ValueError:
            msg = None
        return n_bodies, msg

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
        start = time.time()
        # Reading the _image_size of the data stream we want to read
        n_bytes = self._receive_size()
        if n_bytes == -1:
            msg = "Failed to receive skeleton data!"
            self._logger.warning(msg)
            return list()
        self._logger.debug("Need to receive {} bytes.".format(n_bytes))

        # Reading the data from the socket stream
        try:
            n_bodies, msg = self._receive_skeleton_data(n_bytes=n_bytes)
        except ValueError:
            return list()

        # Convert received data into a list
        try:
            barray = np.fromstring(msg, np.float32)
        except ValueError:
            self._logger.warning("Error when converting raw data to skeleton list!")
            return list()
        bodies = list()
        for body in range(n_bodies):
            cam_space_points = list()
            color_space_points = list()
            depth_space_points = list()
            for i in range(body*self.joint_type_count*7,
                           (body + 1)*self.joint_type_count*7, 7):
                cam_space_points.append((barray[i], barray[i + 1], barray[i + 2]))
                color_space_points.append((barray[i + 3], barray[i + 4]))
                depth_space_points.append((barray[i + 5], barray[i + 6]))
            bodies.append((cam_space_points, color_space_points, depth_space_points))
        self._logger.info("Received skeleton data for {} bod{} in {:.3f} s.".format(
            len(bodies), 'y' if len(bodies) == 1 else 'ies',
            time.time() - start))
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
        data_skeleton = list()
        if self._native_ros:
            # Kinect is connected to the Ubuntu machine, communicate via ROS
            if color:
                img_color = self.color.collect_image()
            if depth:
                img_depth = self.depth.collect_image()
            if skeleton:
                # libfreenect2 does not provide skeleton data
                pass
        else:
            # Kinect is connected to a Windows machine, communicate via
            # TCP/IP socket connection
            if not self._host:
                msg = "No host name for ELTE Kinect Windows tool provided!"
                self._logger.error(msg)
                raise ValueError(msg)
            # Create a TCP/IP socket
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Connect the socket to the host and port where the server listens
            server = self._host, 9999
            self._logger.info('Connect to {} on port {}.'.format(server[0].upper(),
                                                                 server[1]))
            try:
                self._socket.connect(server)
            except socket.gaierror:
                raise RuntimeError("Failed to connect to {}!".format(server[0].upper()) +
                                   " Check your network connection.")
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
            except socket.error as e:
                self._logger.error(str(e))
            finally:
                self._logger.info('Close socket.')
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
        img, dpth, skeletons = self.collect_data(color=True, depth=True, skeleton=True)
        while dpth is None:
            img, dpth, skeletons = self.collect_data(color=True, depth=True, skeleton=True)
        if len(skeletons) != 1:
            raise ValueError("Need to track exactly one person!")
        skeleton = skeletons[0]
        if skeleton is None:
            estimate = None
        else:
            estimate = dict()
            for arm, idx in zip(['left', 'right'],
                                [self.joint_type_hand_left,
                                 self.joint_type_hand_right]):
                cam, color, depth = [a[idx] for a in skeleton]
                # The ELTE KinectOverNetwork tool scales the color image by a
                # factor of 1/2 (to 960x540). We thus adapt the estimated pixel
                # coordinates accordingly.
                color = tuple(x/2.0 if not self._native_ros else x for x in color)

                # TODO: verify this works as expected
                pos = tuple(np.dot(self.trafo, list(cam) + [1])[:-1])
                estimate[arm] = (pos, color, depth)
                # visualize estimate
                ctr = tuple(int(x) for x in color)
                cv2.circle(img, center=ctr, radius=5, color=[255, 0, 0], thickness=3)

                # # validate projection cam 2 pixel
                # ctr2 = tuple(int(x) for x in self.color.projection_camera_to_pixel(position=list(cam)))
                # print ctr, ctr2
                # cv2.circle(img, center=ctr2, radius=2, color=[0, 0, 255], thickness=3)
                # # validate projection pixel 2 cam
                # print 'kinect:', cam
                # z = get_depth(dpth, img.shape[:2], ctr)
                # print 'projct:', self.color.projection_pixel_to_camera(color, z)
            self._pub_vis.publish(img_to_imgmsg(img=img))
        return estimate

    def estimate_object_position(self, img_color, bbox, img_depth):
        """Estimate the approximate position of an object in 3d from a Kinect
        color and corresponding depth image, as well as the bounding box of
        the object detected in the color image.

        :param img_color: A color image.
        :param bbox: The bounding box of the object we are interested in in
            the color image.
        :param img_depth: A depth image corresponding to the color image.
        :return: A list [x, y, z] representing the approximate object position
            in camera coordinates or None if the object was not detected.
        """
        if bbox is None:
            return None

        if len(bbox) == 3:
            # smallest enclosing rectangle
            px, py = bbox[0]
        elif len(bbox) == 4:
            # bounding box
            px, py = bbox[2] - bbox[0], bbox[3] - bbox[1]
        else:
            raise ValueError("Expected rroi or bounding box, got {}!".format(bbox))
        z_3d = get_depth(img_depth, img_color.shape[:2], (px, py))
        pos_cam = self.color.projection_pixel_to_camera(pixel=(px, py), z=z_3d)
        # TODO: verify this works as expected
        pos_rob = np.dot(self.trafo, pos_cam + [1])[:-1]
        return list(pos_rob)

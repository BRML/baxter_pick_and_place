#!/usr/bin/env python

# Copyright (c) 2015--2016, BRML
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
import os

import rospkg
import rospy
from sensor_msgs.msg import CameraInfo


_logger = logging.getLogger('cal_k')
_logger.setLevel(logging.INFO)
_default_loghandler = logging.StreamHandler()
_default_loghandler.setLevel(logging.INFO)
fmt = logging.Formatter('[%(name)s][%(levelname)s][%(asctime)-15s] %(message)s')
_default_loghandler.setFormatter(fmt=fmt)
_logger.addHandler(_default_loghandler)


def get_camera_info(color=True):
    """Retrieve ROS CameraInfo messages from iai_kinect2.

    :param color: Retrieve color (True) or depth (False) camera info.
    :return: A ROS CameraInfo message.
    """
    topic = '/kinect2/{}/camera_info'.format('hd' if color else 'sd')
    msg = None
    try:
        msg = rospy.wait_for_message(topic=topic,
                                     topic_type=CameraInfo,
                                     timeout=1.0)
    except rospy.ROSException:
        _logger.error("Failed to receive {} camera info from topic {}".format(
            'color' if color else 'depth', topic)
        )
    return msg


def get_params_from_msg(msg):
    """Extract the camera parameters from a ROS CameraInfo message.
    1. The camera matrix is defined as
             [fx 0  cx tx]
        cm = [0  fy cy ty].
             [0  0  1  0 ]
    2. The image _image_size given by [height, width].
    3. The distortion coefficients, e.g., [k1, k2, t1, t2, k3].

    :param msg: A ROS CameraInfo message.
    :return: The camera matrix as a 3x3 numpy array, the image _image_size as a 1x2
        numpy array and the distortion coefficients as a 1xX numpy array.
    """
    cm = np.asarray(msg.K, dtype=np.float64).reshape((3, 3))
    size = np.asarray([msg.height, msg.width], dtype=np.uint32)
    dist = np.asarray(msg.D, dtype=np.float64)
    return {'cam_mat': cm, 'size': size, 'dist_coeff': dist}


if __name__ == '__main__':
    """Get the camera parameters for the color and depth sensors in the
    Kinect V2.

    Usage:
        1. Plug the Kinect V2 into an USB 3.0 port of your machine.
        2. From iai_kinect2 run 'roslaunch kinect2_bridge kinect2_bridge.launch'.
        3. Run 'rosrun baxter_pick_and_place calibrate_kinect.py'.
    """
    _logger.info('Initialize node.')
    rospy.init_node('calibrate_kinect_module')
    ns = rospkg.RosPack().get_path('baxter_pick_and_place')

    _logger.info('Query ROS for kinect2 camera info messages.')
    msg_color = get_camera_info(color=True)
    msg_depth = get_camera_info(color=False)

    if msg_color is not None and msg_depth is not None:
        pars_color = get_params_from_msg(msg=msg_color)
        pars_depth = get_params_from_msg(msg=msg_depth)
        pars = {
            'cam_mat_color': pars_color['cam_mat'],
            'size_color': pars_color['size'],
            'dist_coeff_color': pars_color['dist_coeff'],
            'cam_mat_depth': pars_depth['cam_mat'],
            'size_depth': pars_depth['size'],
            'dist_coeff_depth': pars_depth['dist_coeff']
        }
        filename = os.path.join(ns, 'data', 'setup', 'kinect_params.npz')
        _logger.info("Write Kinect V2 color and depth camera matrices to {}.".format(filename))
        np.savez(filename, **pars)
        _logger.info('Done.')
    else:
        _logger.error('Something went wrong! Please try again.')

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

import datetime
import logging
import os
import rospkg

import numpy as np
import rospy
import tf
from sensor_msgs.msg import CameraInfo
from tf import transformations

from core import get_default_handler


class CalibrateKinect(object):
    def __init__(self, log_filename='', log_level=logging.INFO):
        """Get the internal parameters of the color and depth sensors of the
        Kinect V2.

        :param log_filename: The optional file name for the log file.
        :param log_level: The log level to use.
        """
        self._logger = logging.getLogger('cal_kinect')
        self._logger.setLevel(level=logging.DEBUG)
        handlers = get_default_handler(filename=log_filename, level=log_level)
        for handler in handlers:
            self._logger.addHandler(hdlr=handler)

        # In simulation we can read the transform from the ROS master
        self._listener = tf.TransformListener()

    def _lookup_tf_frames(self, from_frame, to_frame):
        """Look up the transform between the target and base frame on the ROS
        master.

        :param from_frame: The base frame.
        :param to_frame: The target frame.
        :return: The requested translation and rotation.
        :raise: ValueError if the requested transform was not found.
        """
        try:
            self._listener.waitForTransform(from_frame,
                                            to_frame,
                                            rospy.Time(),
                                            rospy.Duration(4.0))
            trans, rot = self._listener.lookupTransform(from_frame,
                                                        to_frame,
                                                        rospy.Time(0))
        except (tf.Exception,
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException):
            msg = 'Could not find transform from {} to {} on the ' \
                  'ROS master!'.format(from_frame, to_frame)
            self._logger.warning(msg)
            raise ValueError(msg)
        return trans, rot

    def _check_ros(self):
        """Read the translation and rotation from the ROS master.

        :return: The rotation and translation between RGB and IR sensors.
        """
        trans, rot = self._lookup_tf_frames(
            from_frame='kinect2_rgb_optical_frame',
            to_frame='kinect2_depth_optical_frame')
        return {
            'rotation': transformations.quaternion_matrix(rot)[:-1, :-1],
            'translation': np.array(trans)
        }

    def _get_projection_from_yaml(self, filename):
        """A somewhat ugly hack to parse the iai_kinect2 calib_pose.yaml
        configuration file.

        :param filename: The name of the yaml file to load.
        :return: The rotation and translation between RGB and IR sensors.
        """
        data = dict()
        needline = False
        try:
            with open(filename, 'r') as fp:
                for line in fp:
                    if not line.startswith(' '):
                        needline = False
                        entry = line.split(':')[0]
                        if entry not in data:
                            data[entry] = list()
                    elif line.strip(' ').startswith('data'):
                        needline = True
                        data[entry].append(line.split(':')[1].replace('[', '').rstrip())
                    elif needline:
                        data[entry].append(line.replace(']', '').rstrip())
                        if line.strip(' ').endswith(']'):
                            needline = False
        except IOError:
            self._logger.error("{} not found! Did you copy the calibration "
                               "file to the setup folder?".format(filename))
            return None
        filtered_data = dict()
        for entry in ['rotation', 'translation']:
            filtered_list = list()
            for lst in data[entry]:
                for l in lst.split(','):
                    s = l.strip()
                    if len(s) > 0:
                        filtered_list.append(s)
            filtered_data[entry] = np.array(filtered_list, dtype=np.float64)
        return {
            'rotation': filtered_data['rotation'].reshape((3, 3)),
            'translation': filtered_data['translation']
        }

    def _get_transform(self, filename):
        """Either read the transformation from RGB to  depth sensor from the
        ROS master, or if that fails, from a previously recorded iai_kinect2
        calibration file.

        :param filename: The filename of the previously recorded iai_kinect2
            calibration file.
        :return: The requested transformation.
        """
        try:
            self._logger.info("Try to read RGB--IR transformation from ROS master.")
            trafo = self._check_ros()
        except ValueError:
            self._logger.info("Try to extract RGB--IR transformation from calibration file.")
            trafo = self._get_projection_from_yaml(filename=filename)

        return trafo

    def _get_camera_info(self, color=True):
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
            self._logger.error("Failed to receive {} camera info from topic "
                               "{}".format('color' if color else 'depth',
                                           topic))
        return msg

    @staticmethod
    def _get_params_from_msg(msg):
        """Extract the camera parameters from a ROS CameraInfo message.
        1. The camera matrix is defined as
                 [fx 0  cx tx]
            cm = [0  fy cy ty].
                 [0  0  1  0 ]
        2. The image _image_size given by [height, width].
        3. The distortion coefficients, e.g., [k1, k2, t1, t2, k3].

        :param msg: A ROS CameraInfo message.
        :return: The camera matrix as a 3x3 numpy array, the image size as a
            1x2 numpy array and the distortion coefficients as a 1xX numpy
            array.
        """
        cm = np.asarray(msg.K, dtype=np.float64).reshape((3, 3))
        size = np.asarray([msg.height, msg.width], dtype=np.uint32)
        dist = np.asarray(msg.D, dtype=np.float64)
        return {'cam_mat': cm, 'size': size, 'dist_coeff': dist}

    def calibrate(self, filename):
        """Perform the calibration. First read the internal camera parameters
        for RGB and depth sensor from the ROS master, then try to read
        the transformation from RGB to  depth sensor from either the ROS
        master, or if that fails, from a previously recorded iai_kinect2
        calibration file.

        :param filename: The filename of the previously recorded iai_kinect2
            calibration file.
        :return:
        """
        self._logger.info('Query ROS for kinect2 camera info messages.')
        msg_color = self._get_camera_info(color=True)
        msg_depth = self._get_camera_info(color=False)

        fname = os.path.join(os.path.split(filename)[0], 'calib_pose.yaml')
        trafo = self._get_transform(filename=fname)

        if msg_color is None or msg_depth is None or trafo is None:
            self._logger.error('Something went wrong! Please try again.')
        else:
            pars_color = self._get_params_from_msg(msg=msg_color)
            pars_depth = self._get_params_from_msg(msg=msg_depth)
            pars = {
                'cam_mat_color': pars_color['cam_mat'],
                'size_color': pars_color['size'],
                'dist_coeff_color': pars_color['dist_coeff'],
                'cam_mat_depth': pars_depth['cam_mat'],
                'size_depth': pars_depth['size'],
                'dist_coeff_depth': pars_depth['dist_coeff'],
                'rotation': trafo['rotation'],
                'translation': trafo['translation']
            }
            self._logger.info("Write Kinect V2 internal parameters "
                              "to {}.".format(filename))
            for k, v in pars.iteritems():
                self._logger.info("{}: {}".format(k, v.flatten()))
            np.savez(filename, **pars)
            self._logger.info('Done.')


if __name__ == '__main__':
    """Get the camera parameters for the color and depth sensors in the
    Kinect V2.

    Usage:
        1. Perform iai_kinect2 calibration as detailed here:
            https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration.
        2. Copy the generated calib_pose.yaml file into the setup folder.
        3. Plug the Kinect V2 into an USB 3.0 port of your machine.
        4. Run 'roslaunch kinect2_bridge kinect2_bridge.launch'
           or
            'roslaunch baxter_pick_and_place simulation.launch depth_external:=true'.
        5. Run 'rosrun baxter_pick_and_place calibrate_kinect.py'.
    """
    print 'Initialize node.'
    rospy.init_node('calibrate_kinect_module')
    ns = rospkg.RosPack().get_path('baxter_pick_and_place')

    logfolder = os.path.join(ns, 'log')
    if not os.path.exists(logfolder):
        os.makedirs(logfolder)
    logfile = datetime.datetime.now().strftime(format="%Y%m%d_%H%M")
    logfile = os.path.join(logfolder, '{}_cal_ext.log'.format(logfile))
    logfile = ''

    cal = CalibrateKinect(log_filename=logfile, log_level=logging.INFO)
    filename = os.path.join(ns, 'data', 'setup', 'kinect_parameters.npz')
    cal.calibrate(filename=filename)

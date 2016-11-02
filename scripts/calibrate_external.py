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
from tf import transformations

from calibration import perform_external_calibration
from core import get_default_handler
from simulation.simulation import load_gazebo_model, spawn_gazebo_model, delete_gazebo_model


class ExternalCalibration(object):
    def __init__(self, root_dir, log_filename='', log_level=logging.INFO):
        """Perform the external calibration of the demonstration setup
        consisting of a Kinect V2 camera and a Baxter research robot.

        :param root_dir: Where the baxter_pick_and_place package resides.
        :param log_filename: The optional file name for the log file.
        :param log_level: The log level to use.
        """
        self._root_dir = root_dir
        self._logger = logging.getLogger('cal_ext')
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
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException):
            msg = 'Could not find transform from {} to {} on the ' \
                  'ROS master!'.format(from_frame, to_frame)
            # self._logger.warning(msg)
            raise ValueError(msg)
        return trans, rot

    def _check_ros(self):
        """Read the translation and rotation from the ROS master and convert
        it into a 4x4 affine transformation matrix.

        :return: A 4x4 affine transformation matrix.
        """
        trans, rot = self._lookup_tf_frames(
            from_frame='base', to_frame='kinect2_rgb_optical_frame')
        proj = transformations.quaternion_matrix(rot)
        proj[:-1, -1] = trans
        return proj

    def calibrate(self):
        """Perform the calibration. First try to read the transformation from
         the ROS master. If that fails, resort to performing the calibration
         routine.

        :return:
        """
        # try:
        #     self._logger.info("Try to read external calibration from ROS master.")
        #     trafo = self._check_ros()
        #     raise ValueError
        # except ValueError:
        if True:
            self._logger.info("Perform external calibration.")
            # urdf = load_gazebo_model(os.path.join(self._root_dir, 'models', 'pattern', 'model.urdf'))
            # success = spawn_gazebo_model(urdf, 'acircles', 'object')
            # print success
            trafo = perform_external_calibration(arm='left', n1=3, n2=1,
                                                 root_dir=self._root_dir)

        if trafo is None:
            self._logger.error("Something went wrong! Please try again.")
        else:
            filename = os.path.join(ns, 'data', 'setup', 'external_parameters.npz')
            self._logger.info("Write external parameters to {}.".format(filename))
            np.savez(filename, trafo=trafo)
            self._logger.info("Done.")


def on_shutdown():
    # delete_gazebo_model('acircles')
    pass


if __name__ == '__main__':
    """Get the external camera parameters, i.e., the affine transformation
    relating camera and robot, for the Kinect V2.

    Usage:
        1. Plug the Kinect V2 into an USB 3.0 port of your machine.
        2. Run
            'roslaunch kinect2_bridge kinect2_bridge.launch'
           or
            'roslaunch baxter_pick_and_place simulation.launch depth_external:=true'.
        3. Run 'rosrun baxter_pick_and_place calibrate_external.py'.
    """
    print 'Initialize node.'
    rospy.init_node('calibrate_external_module')
    rospy.on_shutdown(on_shutdown)
    ns = rospkg.RosPack().get_path('baxter_pick_and_place')

    logfolder = os.path.join(ns, 'log')
    if not os.path.exists(logfolder):
        os.makedirs(logfolder)
    logfile = datetime.datetime.now().strftime(format="%Y%m%d_%H%M")
    logfile = os.path.join(logfolder, '{}_cal_ext.log'.format(logfile))
    logfile = ''

    cal = ExternalCalibration(root_dir=ns,
                              log_filename=logfile, log_level=logging.DEBUG)
    cal.calibrate()

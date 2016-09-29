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
from numpy.random import random_sample

import rospy
from std_msgs.msg import Float32MultiArray

from base import Camera
from demo.settings import workspace_limits_m as lims


class Kinect(object):
    def __init__(self):
        self.depth = Camera(topic='/kinect2/sd/image_depth_rect')
        self.color = Camera(topic='/kinect2/hd/image_color_rect')

        self._topic = '/kinect2/skeleton'

    def collect_skeleton(self):
        """Read the latest estimated skeleton data from the Kinect V2
        skeleton topic.

        :return: One of
            - The skeleton as a (n, 3) numpy array of joint positions or
            - None if no skeleton estimate is computed by the Kinect.
        """
        try:
            msg = rospy.wait_for_message(topic=self._topic,
                                         topic_type=Float32MultiArray,
                                         timeout=0.5)
            skeleton = np.asarray(msg.data).reshape((-1, 3))
        except rospy.ROSException:
            # TODO: replace this debugging stuff with 'skeleton = None'
            if random_sample() > 0.5:
                skeleton = np.zeros((10, 3))
            else:
                skeleton = None
        return skeleton

    def estimate_hand_position(self):
        """Extract the estimate for the approximate hand position in 3d
        from the skeleton data given by Kinect.

        :return: One of
            - A list [x, y, z] representing the approximate hand position
                in camera coordinates or
            - None if no skeleton estimate is computed by the Kinect.
        """
        skeleton = self.collect_skeleton()
        if skeleton is None:
            estimate = None
        else:
            # TODO: extract the coordinates from skeleton we are interested in
            estimate = [
                (lims['x_max'] - lims['x_min'])*random_sample() + lims['x_min'],
                (lims['y_max'] - lims['y_min'])*random_sample() + lims['y_min'],
                (lims['z_max'] - lims['z_min'])*random_sample() + lims['z_min']
            ]
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

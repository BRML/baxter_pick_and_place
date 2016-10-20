# -*- coding: utf-8 -*-

"""Module for hardware interfaces.

Implements interfaces to the Baxter research robot (using the Baxter SDK
found at https://github.com/BRML/baxter_interface) and the Kinect V2 in ROS
(using the ROS bridge found at https://github.com/code-iai/iai_kinect2).
"""

from base import (
    img_to_imgmsg,
    imgmsg_to_img
)

from baxter import Baxter

from kinect import Kinect

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


class Camera(object):
    def __init__(self, topic):
        """Base class for a ROS camera.
        A camera should have at least
          - a method to read images from a ROS topic,
          - a method to project pixel coordinates to camera coordinates, and
          - a method to project camera coordinates to pixel coordinates.
        """
        self._topic = topic
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        # use camera_info to get fx, fy, cx, cy?
        self._camera_matrix = np.array([self.fx, 0.0, self.cx, 0.0,
                                        0.0, self.fy, self.cy, 0.0,
                                        0.0, 0.0, 1.0, 0.0,
                                        0.0, 0.0, 1.0, 0.0]).reshape((4, 4))

    def collect_image(self):
        # rospy.waitForMessage(self._topic)
        pass

    def projection_pixel_to_camera(self, pixel):
        raise NotImplementedError()

    def projection_camera_to_pixel(self, position):
        raise NotImplementedError()

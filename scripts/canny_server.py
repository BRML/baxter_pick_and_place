#!/usr/bin/env python

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

import cv2
from dynamic_reconfigure.server import Server
import numpy as np
import rospy

from baxter_pick_and_place.camera_server import BaseCameraServer
from baxter_pick_and_place.cfg import CannyConfig


class CannyServer(BaseCameraServer):

    def __init__(self):
        """ Dynamic reconfigure server for playing with Canny edge detection
        parameters.

        Usage:
        - in a baxter console, do `canny_server.py`.
        - in another baxter console, do
            `rosrun rqt_gui rqt_gui -s reconfigure`.
        """
        super(CannyServer, self).__init__()
        self._c_low = 50
        self._c_high = 190

    def _do_on_callback(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        equ = cv2.equalizeHist(gray)
        cv2.imshow('left camera', equ)
        canny = cv2.Canny(equ, self._c_low, self._c_high, apertureSize=3)
        kernel = np.ones((3, 3), np.uint8)
        canny = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel, iterations=1)
        cv2.imshow('left canny', canny)
        cv2.waitKey(3)

    def _update(self, config):
        s = ""
        if self._c_low != config['canny_low']:
            s += 'cl: %i -> %i' % (self._c_low, config['canny_low'])
            self._c_low = config['canny_low']
        if self._c_high != config['canny_high']:
            s += 'ch: %i -> %i' % (self._c_high, config['canny_high'])
            self._c_high = config['canny_high']
        if len(s) > 0:
            rospy.loginfo(s)


if __name__ == "__main__":
    rospy.init_node("canny", anonymous=True)

    cs = CannyServer()
    rospy.on_shutdown(cs.stop)
    srv = Server(CannyConfig, cs.rec_callback)
    cs.start()
    rospy.spin()

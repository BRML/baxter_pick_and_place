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
import rospy

from baxter_pick_and_place.camera_server import BaseCameraServer
from baxter_pick_and_place.cfg import CamConfig


class CameraServer(BaseCameraServer):

    def __init__(self):
        """ Dynamic reconfigure server for playing with baxter camera
        parameters.

        Usage:
        - in a baxter console, do `cam_config_server.py`.
        - in another baxter console, do
            `rosrun rqt_gui rqt_gui -s reconfigure`.
        """
        super(CameraServer, self).__init__()
        self._exposure = -1
        self._gain = 0
        self._wb_red = -1
        self._wb_green = -1
        self._wb_blue = -1

    def _do_on_callback(self, image):
        cv2.imshow('left camera', image)
        cv2.waitKey(3)

    def _update(self, config):
        s = ""
        if self._exposure != config['exposure']:
            s += 'exp: %i -> %i' % (self._exposure, config['exposure'])
            self._camera.exposure = config['exposure']
            self._exposure = config['exposure']
        if self._gain != config['gain']:
            s += 'gain: %i -> %i' % (self._gain, config['gain'])
            self._camera.gain = config['gain']
            self._gain = config['gain']
        if self._wb_red != config['wb_red']:
            s += 'red: %i -> %i' % (self._wb_red, config['wb_red'])
            self._camera.white_balance_red = config['wb_red']
            self._wb_red = config['wb_red']
        if self._wb_green != config['wb_green']:
            s += 'green: %i -> %i' % (self._wb_green, config['wb_green'])
            self._camera.white_balance_green = config['wb_green']
            self._wb_green = config['wb_green']
        if self._wb_blue != config['wb_blue']:
            s += 'blue: %i -> %i' % (self._wb_blue, config['wb_blue'])
            self._camera.white_balance_blue = config['wb_blue']
            self._wb_blue = config['wb_blue']
        if len(s) > 0:
            rospy.loginfo(s)


if __name__ == "__main__":
    rospy.init_node("camera", anonymous=True)

    cs = CameraServer()
    rospy.on_shutdown(cs.stop)
    srv = Server(CamConfig, cs.rec_callback)
    cs.start()
    rospy.spin()

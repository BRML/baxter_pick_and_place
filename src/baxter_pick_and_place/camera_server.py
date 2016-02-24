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
import rospy

from sensor_msgs.msg import Image

from baxter_pick_and_place.baxter_robot import BaxterRobot
from baxter_pick_and_place.image import cut_imgmsg, imgmsg2img
from baxter_pick_and_place.settings import parameters as table
from baxter_pick_and_place.settings import top_pose


class BaseCameraServer(BaxterRobot):

    def __init__(self):
        """ Base class for dynamic reconfigure camera server.
        See http://wiki.ros.org/dynamic_reconfigure/Tutorials/
            SettingUpDynamicReconfigureForANode%28python%29.
        """
        self._arm = 'left'
        super(BaseCameraServer, self).__init__(limb=self._arm)

        self._cam_sub = None
        self._top_pose = top_pose

        self._limb.move_to_neutral()

    def _update(self, config):
        """ Implement what should be done with the elements of 'config' when
        dynamic reconfigure parameters are changed.
        """
        pass

    def start(self):
        print 'Starting dynamic reconfigure server ...'
        if self.move_to_pose(self._top_pose):
            s = '/cameras/' + self._arm + '_hand_camera/image'
            self._cam_sub = rospy.Subscriber(s, Image,
                                             callback=self._camera_callback)
            print "Press 'Ctrl+C' to stop."
        else:
            print 'Failed motion'

    def _camera_callback(self, data):
        """ Callback routine for the camera subscriber. """
        self._do_on_callback(imgmsg2img(cut_imgmsg(data, **table)))

    def _do_on_callback(self, image):
        """ Implement what should be done with the image during each camera
        callback.
        :param image: an image
        """
        pass

    def clean_shutdown(self):
        print 'Exiting dynamic reconfigure server ...'
        if self._cam_sub is not None:
            self._cam_sub.unregister()
        self._limb.move_to_neutral()
        if not self._init_state:
            print "Disabling robot..."
            self._rs.disable()
        cv2.destroyAllWindows()
        return True

    def stop(self):
        return self.clean_shutdown()

    def rec_callback(self, config, level):
        """ Callback routine for the reconfiguration server subscriber. """
        self._update(config)
        return config

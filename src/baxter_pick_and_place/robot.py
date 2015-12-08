# Copyright (c) 2015, BRML
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

import baxter_interface
from baxter_interface import CHECK_VERSION


class Robot(object):

    def __init__(self):
        print("\nGetting robot state ... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def clean_shutdown(self):
        print("\nExiting demonstrator ...")
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def pick_and_place_object(self):
        print ' rabbiting away ...'
        # Wait for object to be triggered
        # move limb to top-down-view pose
        # record top-down-view image
        # detect object candidates
        # for candidate in candidates:
        #   move limb to candidate + offset
        #   record candidate image
        #   select 200x200 patch
        #   call service
        # make decision
        # compute object pose (if not already done)
        # move limb to object pose
        # grasp object
        # move limb to target location
        # release object
        # move limb to neutral configuration
        return True

    def _move_to_pose(self, pose):
        return True

    def _perturbe_pose(self):
        return True

    def _record_image(self):
        img = None
        return img

    def _grasp_object(self):
        return True

    def _release_object(self):
        return True

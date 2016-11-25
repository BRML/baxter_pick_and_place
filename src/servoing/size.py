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

from base import Servoing


class ServoingSize(Servoing):
    def _tolerance(self):
        """The tolerance required to achieve for accepting a grasp pose."""
        return 5*self._tol

    def estimate_distance(self, object_id, rroi, arm):
        """Estimate the distance to the object from
        - the known size of the object in meters,
        - the meters per pixel at one meter reference distance and
        - the approximate size of the object in pixels
        by computing
            distance = size_meters / (size_pixels * mpp).

        :param object_id: The object identifier of the object to estimate the
            distance to.
        :param rroi: The rotated rectangle enclosing the segmented object,
            given by ((cx, cy), (w, h), alpha).
        :param arm: The arm <'left', 'right'> to control.
        :return: The approximate distance from the gripper to the object.
        """
        size_meters = self._object_size_meters[object_id]
        size_pixels_at_gripper = size_meters/abs(self._robot.cam_offset[2])/self._robot.cameras[arm].meters_per_pixel
        size_pixels = min(rroi[1])
        if size_pixels < 1.3*size_pixels_at_gripper:
            dist_cam_obj = size_meters/(size_pixels*self._robot.cameras[arm].meters_per_pixel)
        else:
            dist_cam_obj = 0.0
        return dist_cam_obj

    def correct_height(self, arm):
        """Make sure the gripper height is appropriate before attempting to
        grasp the object.

        :param arm: The arm <'left', 'right'> to control.
        :return: A boolean success value.
        """
        return True

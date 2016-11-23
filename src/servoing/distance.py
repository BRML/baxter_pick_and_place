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


class ServoingDistance(Servoing):
    def estimate_distance(self, object_id, rroi, arm):
        """Estimate the distance to the object from
        - the measured pose of the end effector and
        - the measured height of the table top in robot coordinates
        by computing
            distance = height_gripper - height_table.

        :param object_id: The object identifier of the object to estimate the
            distance to.
        :param rroi: The rotated rectangle enclosing the segmented object,
            given by ((cx, cy), (w, h), alpha).
        :param arm: The arm <'left', 'right'> to control.
        :return: The approximate distance from the gripper to the object.
        """
        return self._robot.endpoint_pose(arm=arm)[2] - self._robot.z_table

    def correct_height(self, arm):
        """Make sure the gripper height is appropriate before attempting to
        grasp the object.

        :param arm: The arm <'left', 'right'> to control.
        :return: A boolean success value.
        """
        pose = self._robot.endpoint_pose(arm=arm)
        pose[2] = self._robot.z_table + 0.01
        try:
            cfg = self._robot.ik(arm=arm, pose=pose)
            self._robot.move_to_config(config=cfg)
        except ValueError as e:
            self._logger.error(e)
            return False
        return True

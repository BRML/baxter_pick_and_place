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

from tf import transformations
from geometry_msgs.msg import Pose


def list_to_pose_msg(pose):
    """Create a pose ROS message.

        :param pose: The pose to stamp. One of
            - a list of length 6 [x, y, z, roll, pitch, yaw] or
            - a list of length 7 [x, y, z, qx, qy, qz, qw].
        :return: A pose ROS message.
        """
    if isinstance(pose, list):
        if len(pose) == 6 or len(pose) == 7:
            msg = Pose()
            msg.position.x, msg.position.y, msg.position.z = pose[:3]
            if len(pose) == 6:
                q = transformations.quaternion_from_euler(pose[3], pose[4], pose[5])
            else:
                q = pose[3:]
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
            return msg
        raise ValueError("Expected pose to be [x, y, z, r, p, y] or [x, y, z, qx, qy, qz, qw]!")
    raise ValueError("Expected pose to be a list of length 6 or 7!")


def pose_msg_to_list(pose):
    """Convert a pose ROS message into a list.

    :param pose: A pose ROS message.
    :return: The pose as a list [x, y, z, roll, pitch, yaw].
    """
    if isinstance(pose, Pose):
        rot = list(transformations.euler_from_quaternion(pose['orientation']))
        return pose['position'] + rot
    raise ValueError("Expected pose to be a ROS Pose message!")


def pose_dict_to_list(pose):
    """Convert a pose baxter_interface dictionary into a list.

    :param pose: A pose baxter_interface dict.
    :return: The pose as a list [x, y, z, roll, pitch, yaw].
    """
    if isinstance(pose, dict):
        pos = [pose['position'].x, pose['position'].y, pose['position'].z]
        rot = list(transformations.euler_from_quaternion(pose['orientation']))
        return pos + rot
    raise ValueError("Expected pose to be a dictionary!")


def pose_dict_to_hom(pose):
    """Convert a pose baxter_interface dictionary into a homogeneous
    transformation matrix.

    :param pose: A pose baxter_interface dict.
    :return: The corresponding homogeneous transformation matrix (a 4x4
        numpy array).
    """
    if isinstance(pose, dict):
        hom = transformations.quaternion_matrix(pose['orientation'])
        hom[:-1, -1] = pose['position']
        return hom
    raise ValueError("Expected pose to be a dictionary!")


def hom_to_list(matrix):
    """Convert a homogeneous transformation matrix into a list.

    :param matrix: A 4x4 numpy array representing a homogeneous
        transformation matrix.
    :return: The corresponding pose as a list [x, y, z, roll, pitch, yaw].
    """
    if matrix.shape == (4, 4):
        rot = list(transformations.euler_from_matrix(matrix[:-1, :-1]))
        return list(matrix[:-1, -1]) + rot
    raise ValueError("Expected a 4x4 numpy array!")

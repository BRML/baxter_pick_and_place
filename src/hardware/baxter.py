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


import logging

import rospy
from tf import transformations

from geometry_msgs.msg import (
    Pose,
    PoseStamped
)

import baxter_interface
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

from base import Camera
from motion_planning.base import MotionPlanner


# Set up logging
_logger = logging.getLogger('baxter')
_logger.setLevel(logging.INFO)
_default_loghandler = logging.StreamHandler()
_default_loghandler.setLevel(logging.INFO)
_default_loghandler.setFormatter(logging.Formatter('[%(name)s][%(levelname)s] %(message)s'))
_logger.addHandler(_default_loghandler)


def remove_default_loghandler():
    """Call this to mute this library or to prevent duplicate messages
    when adding another log handler to the logger named 'baxter'."""
    _logger.removeHandler(_default_loghandler)


class Baxter(object):
    def __init__(self):
        self._arms = ['left', 'right']
        self._limbs = {a: baxter_interface.Limb(a)
                       for a in self._arms}
        self._grippers = {a: baxter_interface.Gripper(a)
                          for a in self._arms}
        self._cameras = {a: Camera(topic='/cameras/{}_hand_camera/image'.format(a))
                         for a in self._arms}
        self._rs = None
        self._init_state = None

    def set_up(self):
        _logger.info("Getting robot state")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        _logger.info("Enabling robot")
        self._rs.enable()

        for arm in self._arms:
            _logger.info("Moving limbs to neutral configuration")
            self._limbs[arm].move_to_neutral()
            _logger.info("Calibrating grippers")
            self._grippers[arm].calibrate()
            # self._camera.resolution = (1280, 800)
            # self._camera.fps = 14.0

    def clean_up(self):
        _logger.info("Initiating safe shut-down")
        for arm in self._arms:
            self._grippers[arm].open()
            _logger.info("Moving limbs to neutral configuration")
            self._limbs[arm].move_to_neutral()
        if not self._init_state:
            _logger.info("Disabling robot")
            self._rs.disable()

    @staticmethod
    def _stamp_pose(pose, target_frame='base'):
        """Create a stamped pose ROS message.

        :param pose: The pose to stamp. One of
            - a ROS Pose,
            - a list of length 6 [x, y, z, roll, pitch, yaw] or
            - a list of length 7 [x, y, z, qx, qy, qz, qw].
        :param target_frame: The name of the target frame.
        :return: A stamped pose ROS message.
        """
        msg = Pose()
        if isinstance(pose, Pose):
            msg = pose
        elif isinstance(pose, list):
            if len(pose) == 6 or len(pose) == 7:
                msg.position.x, msg.position.y, msg.position.z = pose[:3]
                if len(pose) == 6:
                    q = transformations.quaternion_from_euler(pose[3], pose[4], pose[5])
                else:
                    q = pose[3:]
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
            else:
                s = "Expected pose to be [x, y, z, r, p, y] or [x, y, z, qx, qy, qz, qw]!"
                _logger.error(s)
                raise ValueError(s)
        else:
            s = "Expected pose to be a Pose or list of length 6 or 7!"
            _logger.error(s)
            raise ValueError(s)
        pose_msg = PoseStamped()
        pose_msg.pose = msg
        pose_msg.header.frame_id = target_frame
        pose_msg.header.stamp = rospy.Time.now()
        return pose_msg

    def _inverse_kinematics(self, arm, pose=None):
        """Solve inverse kinematics for one limb at given pose.

        :param arm: The arm <'left', 'right'> to control.
        :param pose:  The pose to stamp. One of
            - None, in which case the current set of joint angles is returned,
            - a ROS Pose,
            - a list of length 6 [x, y, z, roll, pitch, yaw] or
            - a list of length 7 [x, y, z, qx, qy, qz, qw].
        :return:
        """
        if pose is None:
            return self._limbs[arm].joint_angles()

        pq = self._stamp_pose(pose, target_frame="base")
        node = "ExternalTools/" + arm + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        ik_request.pose_stamp.append(pq)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            _logger.error("Service request failed: %r" % (error_message,))
            raise

        if ik_response.isValid[0]:
            # convert response to joint position control dictionary
            return dict(zip(ik_response.joints[0].name,
                            ik_response.joints[0].position))
        else:
            s = "inverse kinematics - No valid joint configuration found"
            _logger.error('{} for {} arm at {}'.format(s, arm, pose))
            raise ValueError(s)

    def control(self, arm, trajectory):
        """Control one limb using position, velocity or torque control.

        :param arm: The arm <'left', 'right'> to control.
        :param trajectory: A generator MotionPlanner instance.
        :return:
        """
        if not isinstance(trajectory, MotionPlanner):
            raise TypeError("'trajectory' must be a MotionPlanner instance!")
        if trajectory.controller_type == 'position':
            for q in trajectory:
                self._limbs[arm].set_joint_positions(q)
        elif trajectory.controller_type == 'velocity':
            for v in trajectory:
                self._limbs[arm].set_joint_velocities(v)
        elif trajectory.controller_type == 'torque':
            for t in trajectory:
                self._limbs[arm].set_joint_torques(t)
        else:
            raise KeyError("No such control mode: '{}'!".format(trajectory.controller_type))

    def grasp(self, arm):
        """Close the specified gripper and validate that it grasped something.
        Blocking command.

        :param arm: The arm <'left', 'right'> to control.
        :return: bool describing if the position move has been preempted by a
            position command exceeding the moving_force threshold denoting a
            grasp.
        """
        self._grippers[arm].close(block=True)
        return self._grippers[arm].gripping()

    def release(self, arm):
        """Open the specified gripper. Blocking command.

        :param arm: The arm <'left', 'right'> to control.
        :return:
        """
        return self._grippers[arm].open(block=True)

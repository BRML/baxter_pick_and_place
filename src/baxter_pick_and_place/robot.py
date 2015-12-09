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

import numpy as np
import time

import rospy

from tf import transformations
from geometry_msgs.msg import (
    Pose,
    PoseStamped
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

import baxter_interface
from baxter_interface import CHECK_VERSION


class Robot(object):

    def __init__(self):
        """
         A baxter research robot instance with some additional functionality.
        """
        self._limbs = {
            'left': baxter_interface.Limb('left'),
            'right': baxter_interface.Limb('right')
        }
        self._top_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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
        """ Detect, pick up and place an object upon receiving an execution
        command.

        :return: boolean flag on completion
        """
        print ' rabbiting away ...'
        # Wait for object to be triggered---dummy

        def _trigger_dummy(max_time):
            t = np.random.uniform(0.0, max_time)
            time.sleep(t)
            print '  an object was triggered after %.2fs' % t

        _trigger_dummy(10.0)
        # move limb to top-down-view pose
        self._move_to_pose('left', self._top_pose)
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
        if np.random.random() > 0.2:
            print '  object placed successfully'
            return True
        print '  something went wrong'
        return False

    def _move_to_pose(self, limb, pose):
        """ Move robot limb to Cartesian pose.
        :type limb: str
        :param limb: the limb of the robot, <'left', 'right'>
        :type pose: [float, float, float, float, float, float]
        :param pose: desired Cartesian pose
        :return: boolean flag on completion
        """
        try:
            cmd = self._inverse_kinematics(limb, pose)
        except Exception as e:
            print e
            ret = False
        else:
            ret = self._limbs[limb].move_to_joint_positions(cmd)
        return ret

    def _perturbe_pose(self):
        return True

    def _record_image(self):
        img = None
        return img

    def _grasp_object(self):
        return True

    def _release_object(self):
        return True

    def _inverse_kinematics(self, limb, pose=None):
        """ Inverse kinematics of one Baxter arm.
        Compute valid set of joint angles for a given Cartesian pose using the
        inverse kinematics service. Return current joint angles if None pose is
        given.
        :type limb: str
        :param limb: the limb of the robot, <'left', 'right'>
        :type pose: [float, float, float, float, float, float]
        :param pose: desired Cartesian pose
        :return: valid set of joint angles
        """
        if pose is None:
            return self._limbs[limb].joint_angles()

        qp = list_to_pose_stamped(pose, "base")

        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        ik_request.pose_stamp.append(qp)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            raise

        if ik_response.isValid[0]:
            # convert response to joint position control dictionary
            return dict(zip(ik_response.joints[0].name,
                            ik_response.joints[0].position))
        else:
            rospy.logerr("ERROR - inverse kinematics - No valid joint configuration found")
            raise Exception("ERROR - inverse kinematics - No valid joint configuration found")


def list_to_pose(pose_list):
    pose_msg = Pose()
    if len(pose_list) == 7:
        pose_msg.position.x = pose_list[0]
        pose_msg.position.y = pose_list[1]
        pose_msg.position.z = pose_list[2]
        pose_msg.orientation.x = pose_list[3]
        pose_msg.orientation.y = pose_list[4]
        pose_msg.orientation.z = pose_list[5]
        pose_msg.orientation.w = pose_list[6]
    elif len(pose_list) == 6:
        pose_msg.position.x = pose_list[0]
        pose_msg.position.y = pose_list[1]
        pose_msg.position.z = pose_list[2]
        q = transformations.quaternion_from_euler(pose_list[3], pose_list[4],
                                                  pose_list[5])
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]
    else:
        print "Expected either 6 or 7 elements in list: (x,y,z,r,p,y) or (x,y,z,qx,qy,qz,qw)"
    return pose_msg


def list_to_pose_stamped(pose_list, target_frame):
    pose_msg = PoseStamped()
    pose_msg.pose = list_to_pose(pose_list)
    pose_msg.header.frame_id = target_frame
    pose_msg.header.stamp = rospy.Time.now()
    return pose_msg

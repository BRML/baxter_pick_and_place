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
        self._top_pose = [
            0.50,  # x = front back
            0.00,  # y = left right
            0.15,  # z = up down
            -1.0*np.pi,  # roll = horizontal
            0.0*np.pi,  # pitch = vertical
            0.0*np.pi  # yaw = rotation
        ]
        self._N_TRIES = 2

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

        limb = 'left'
        # move limb to top-down-view pose
        # try up to 3 times to find a valid pose
        if not self._move_to_pose(limb, self._top_pose):
            n_tries = self._N_TRIES
            while n_tries > 0:
                print '  ', n_tries, 'more tries'
                n_tries -= 1
                if self._perturbe_pose(limb, self._top_pose):
                    print 'perturbation worked'
                    n_tries = -1
            if not n_tries == -1:
                return False
        return self._try_object(limb)

    def _try_object(self, limb):
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
        :type pose: {'position': (x, y, z), 'orientation': (a, b, c)}
        :param pose: desired Cartesian pose
        :return: boolean flag on completion
        """
        try:
            cmd = self._inverse_kinematics(limb, pose)
        except Exception as e:
            print e
            return False
        else:
            self._limbs[limb].move_to_joint_positions(cmd)
        return True

    def _perturbe_pose(self, limb, pose):
        """ Add a small perturbation to the Cartesian pose of the robot.
        :type limb: str
        :param limb: the limb of the robot, <'left', 'right'>
        :type pose: {'position': (x, y, z), 'orientation': (a, b, c)}
        :param pose: desired Cartesian pose
        :return: boolean flag on completion
        """
        perturbation = [
            np.random.random()*0.02,
            np.random.random()*0.03,
            np.random.random()*0.02,
            np.random.random()*2.0/180.0,
            np.random.random()*2.0/180.0,
            np.random.random()*2.0/180.0
        ]
        p = [a + b for a, b in zip(pose, perturbation)]
        print pose
        print perturbation
        print p
        return self._move_to_pose(limb, p)

    def _endpoint_pose(self, limb):
        qp = self._limbs[limb].endpoint_pose()
        r = transformations.euler_from_quaternion(qp['orientation'])
        return [qp['position'][0], qp['position'][1], qp['position'][2],
                r[0], r[1], r[2]]

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
        :type pose: {'position': (x, y, z), 'orientation': (a, b, c)}
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
    """ Pose list to ROS Pose msg.
    :param pose_list: pose list
    :return: ROS Pose message
    """
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
    """ Pose list to ROS PoseStamped msg.
    :param pose_list: pose list
    :param target_frame: name of the target frame
    :return: ROS PoseStamped message
    """
    pose_msg = PoseStamped()
    pose_msg.pose = list_to_pose(pose_list)
    pose_msg.header.frame_id = target_frame
    pose_msg.header.stamp = rospy.Time.now()
    return pose_msg


def pose_to_list(pose):
    """ Baxter pose to pose list.
    :type pose: {'position': (x, y, z), 'orientation': (a, b, c)}
    :param pose: baxter pose
    :return: pose list
    """
    return [pose['position'][0], pose['position'][1], pose['position'][2],
            pose['orientation'][0], pose['orientation'][1], pose['orientation'][2]]

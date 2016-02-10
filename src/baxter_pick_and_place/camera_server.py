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

import baxter_interface
from baxter_interface import CHECK_VERSION
import cv2
import rospy
from tf import transformations

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)
from geometry_msgs.msg import (
    Pose,
    PoseStamped
)
from sensor_msgs.msg import Image

from baxter_pick_and_place.image import cut_imgmsg, _imgmsg2img
from baxter_pick_and_place.settings import parameters as table
from baxter_pick_and_place.settings import top_pose


class BaseCameraServer(object):

    def __init__(self):
        """ Base class for dynamic reconfigure camera server.
        See http://wiki.ros.org/dynamic_reconfigure/Tutorials/
            SettingUpDynamicReconfigureForANode%28python%29.
        """
        self._cam = baxter_interface.CameraController('left_hand_camera')
        self._cam_sub = None

        self._limb = baxter_interface.Limb('left')
        self._top_pose = top_pose

        print "\nGetting robot state ... "
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print "Enabling robot... "
        self._rs.enable()

        self._limb.move_to_neutral()
        self._cam.resolution = (1280, 800)
        self._cam.fps = 14.0
        self._cam.exposure = 50
        self._cam.gain = 0
        self._cam.white_balance_red = -1
        self._cam.white_balance_green = -1
        self._cam.white_balance_blue = -1

    def _update(self, config):
        """ Implement what should be done with the elements of 'config' when
        dynamic reconfigure parameters are changed.
        """
        pass

    def start(self):
        print 'Starting dynamic reconfigure server ...'
        if self._move_to_pose(self._top_pose):
            s = '/cameras/left_hand_camera/image'
            self._cam_sub = rospy.Subscriber(s, Image,
                                             callback=self._cam_callback)
            print "Press 'Ctrl+C' to stop."
        else:
            print 'Failed motion'

    def _cam_callback(self, data):
        """ Callback routine for the camera subscriber. """
        self._do_on_callback(_imgmsg2img(cut_imgmsg(data, **table)))

    def _do_on_callback(self, image):
        """ Implement what should be done with the image during each camera
        callback.
        :param image: an image
        """
        pass

    def stop(self):
        print 'Exiting dynamic reconfigure server ...'
        if self._cam_sub is not None:
            self._cam_sub.unregister()
        self._limb.move_to_neutral()
        if not self._init_state:
            print "Disabling robot..."
            self._rs.disable()
        cv2.destroyAllWindows()
        return True

    def rec_callback(self, config, level):
        """ Callback routine for the reconfiguration server subscriber. """
        self._update(config)
        return config

    def _move_to_pose(self, pose):
        """ Move robot limb to Cartesian pose.
        :type pose: [float, float, float, float, float, float]
        :param pose: desired Cartesian pose
        :return: boolean flag on completion
        """
        try:
            cmd = self._inverse_kinematics(pose)
        except Exception as e:
            print e
            return False
        self._limb.move_to_joint_positions(cmd)
        return True

    def _inverse_kinematics(self, pose=None):
        """ Inverse kinematics of one Baxter arm.
        Compute valid set of joint angles for a given Cartesian pose using the
        inverse kinematics service. Return current joint angles if None pose is
        given.
        :type pose: {'position': (x, y, z), 'orientation': (a, b, c)}
        :param pose: desired Cartesian pose
        :return: valid set of joint angles
        """
        if pose is None:
            return self._limb.joint_angles()

        qp = list_to_pose_stamped(pose, "base")

        node = "ExternalTools/left/PositionKinematicsNode/IKService"
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
            s = "inverse kinematics - No valid joint configuration found"
            rospy.logerr(s)
            raise Exception(s)


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

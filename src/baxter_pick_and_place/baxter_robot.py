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

import baxter_interface
import rospy
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)
from baxter_interface import CHECK_VERSION
from geometry_msgs.msg import (
    Pose,
    PoseStamped
)
from sensor_msgs.msg import Image
from std_srvs.srv import Empty

from visual.image import (
    resize_imgmsg,
    string2imgmsg
)


class BaxterRobot(object):

    def __init__(self, limb):
        """
         A baxter research robot instance with some additional functionality.
         :param limb: The limb to use for the demonstration.
        """
        self._arm = limb

        self._limb = baxter_interface.Limb(self._arm)
        self._gripper = baxter_interface.Gripper(self._arm)
        # camera handling is one fragile thing ...
        reset_srv = rospy.ServiceProxy('cameras/reset', Empty)
        rospy.wait_for_service('cameras/reset', timeout=10)
        reset_srv()
        try:
            baxter_interface.CameraController('head_camera').close()
        except AttributeError:
            pass
        self._camera = baxter_interface.CameraController(self._arm +
                                                         '_hand_camera')

        self._display_pub = rospy.Publisher('/robot/xdisplay', Image,
                                            queue_size=10, latch=True)

        print "\nGetting robot state ... "
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print "Enabling robot... "
        self._rs.enable()

        self._limb.move_to_neutral()
        self._gripper.calibrate()
        self._camera.resolution = (1280, 800)
        self._camera.fps = 14.0
        # http://www.productionapprentice.com/featured/the-truth-about-video-gain-and-how-to-use-it-properly/
        self._camera.exposure = 50
        self._camera.gain = 0
        self._camera.white_balance_red = -1
        self._camera.white_balance_green = -1
        self._camera.white_balance_blue = -1

    def clean_shutdown(self):
        """ Implement clean shutdown of the robot. """
        pass

    """ =======================================================================
        Helper functions and kinematics
    ======================================================================= """
    def _camera_callback(self, data):
        """ Implement callback routine for the camera subscriber. """
        pass

    def display_imgmsg(self, imgmsg):
        """ Display an image on the screen of the robot.
        :param imgmsg: a ROS image message
        """
        try:
            self._display_pub.publish(resize_imgmsg(imgmsg))
        except TypeError:
            rospy.logerr('display_imgmsg - ' +
                         'Something is wrong with the ROS image message.')

    def display_text(self, s1, s2=None):
        """ Display up to two lines of text on the screen of the robot.
        :param s1: the first line of text
        :param s2: the (optional second line of text
        """
        self.display_imgmsg(string2imgmsg(s1, s2))

    def grasp_object(self):
        """ Close the gripper and validate that it grasped something.
        :return: bool describing if the position move has been preempted by a
        position command exceeding the moving_force threshold denoting a grasp
        """
        self._gripper.close(block=True)
        return self._gripper.gripping()

    def release_object(self):
        """ Open the gripper. """
        self._gripper.open(block=True)

    def move_to_pose(self, pose, verbose=False):
        """ Move robot limb to Cartesian pose.
        :type pose: [float, float, float, float, float, float]
        :param pose: desired Cartesian pose
        :param verbose: whether to print debugging information
        :return: boolean flag on completion
        """
        if verbose:
            rospy.loginfo('Move to x: %.3fm, y: %.3fm, z: %.3fm, c: %.3frad' %
                          (pose[0], pose[1], pose[2], pose[5]))
        try:
            cmd = self._inverse_kinematics(pose)
        except ValueError:
            return False
        self._limb.move_to_joint_positions(cmd)
        if verbose:
            pose = self._endpoint_pose()
            rospy.loginfo('Arrd at x: %.3fm, y: %.3fm, z: %.3fm, c: %.3frad' %
                          (pose[0], pose[1], pose[2], pose[5]))
        return True

    def _modify_pose(self, offset, pose=None):
        """ Modify the given/current pose by an additive offset.
        :param offset: the offset [dx, dy, dz, da, db, dc] to add
        :param pose: the given pose, or None if current pose is to be used
        :return: modified pose [x, y, z, a, b, c]
        """
        if pose is None:
            pose = self._endpoint_pose()
        return [a + b for a, b in zip(pose, offset)]

    def _endpoint_pose(self):
        """ Current pose of the wrist of one arm of the baxter robot.
        :return: pose [x, y, z, a, b, c]
        """
        qp = self._limb.endpoint_pose()
        r = transformations.euler_from_quaternion(qp['orientation'])
        return [qp['position'][0], qp['position'][1], qp['position'][2],
                r[0], r[1], r[2]]

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

        node = "ExternalTools/" + self._arm + "/PositionKinematicsNode/IKService"
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
            raise ValueError(s)


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
        print "Expected either 6 or 7 elements in list: " + \
            "(x,y,z,r,p,y) or (x,y,z,qx,qy,qz,qw)"
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

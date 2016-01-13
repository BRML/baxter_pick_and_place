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
import string
import time

import rospy

from tf import transformations
from sensor_msgs.msg import Image
from geometry_msgs.msg import (
    Pose,
    PoseStamped
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)
from std_srvs.srv import Empty

import baxter_interface
from baxter_interface import CHECK_VERSION

from baxter_pick_and_place.image import (
    detect_object_candidates,
    select_image_patch,
    find_calibration_pattern,
    resize_imgmsg
)


class Robot(object):

    def __init__(self, limb):
        """
         A baxter research robot instance with some additional functionality.
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
        self._camera = baxter_interface.CameraController(self._arm + '_hand_camera')
        self._imgmsg = None
        self._cam_params = None
        self._top_pose = [
            0.50,  # x = front back
            0.00,  # y = left right
            0.15,  # z = up down
            -1.0*np.pi,  # roll = horizontal
            0.0*np.pi,  # pitch = vertical
            0.0*np.pi  # yaw = rotation
        ]
        self._N_TRIES = 2
        self._N_IMGS_CALIB = 2

        print "\nGetting robot state ... "
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print "Enabling robot... "
        self._rs.enable()

        self._limb.set_joint_position_speed(0.5)
        self._limb.move_to_neutral()
        self._gripper.calibrate()
        self._camera.resolution = (1280, 800)

    def clean_shutdown(self):
        """ Clean shutdown of the robot.
        :return: True on completion
        """
        print "\nExiting demonstrator ..."
        self._limb.move_to_neutral()
        if not self._init_state:
            print "Disabling robot..."
            self._rs.disable()
        return True

    def write_setup(self, setup_file):
        """ Perform the camera calibration of the robot's hand cameras.
        :param setup_file: the file to write the calibration into.
        """
        pose = [
            0.60,
            0.20,
            0.15,
            -1.0*np.pi,
            0.0*np.pi,
            0.0*np.pi
        ]

        def getc():
            """ Gets a single character from standard input. Does not echo to
            the screen.
            See http://code.activestate.com/recipes/134892/.
            :return: a single character
            """
            import sys
            import tty
            import termios
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

        n_imgs_calib = 0
        while n_imgs_calib < self._N_IMGS_CALIB:
            print "Press 'r' to record an image."
            k = getc()
            if k == 'r':
                imgmsg = self._record_image()
                self._display_image(imgmsg)
                n_imgs_calib += 1
                ret, objp, imgp = find_calibration_pattern(imgmsg,
                                                           verbose=True)
                print "Recorded %i of %i images." % (n_imgs_calib,
                                                     self._N_IMGS_CALIB)
            else:
                pass
        # calibrate camera
        # store parameters

        # setup = dict()
        # for limb in self._limbs:
        #     setup[limb] = dict()
        #     # setup pose
        #     setup[limb]['pose'] = pose
        #     # distance camera--table
        #     # TODO: make more robust
        #     self._move_to_pose(limb, pose)
        #     d = baxter_interface.analog_io.AnalogIO(limb + '_hand_range').state()
        #     setup[limb]['distance'] = float(d/1000.0)
        #     # camera resolution
        #     setup[limb]['width'], setup[limb]['height'] = \
        #         self._cameras[limb].resolution
        #     # The following values are taken from
        #     # http://sdk.rethinkrobotics.com/wiki/Worked_Example_Visual_Servoing
        #     # meters per pixel at 1 m
        #     setup[limb]['mpp'] = 0.0025
        #     # camera offset in x
        #     setup[limb]['ox'] = 0.01
        #     # camera offset in y
        #     setup[limb]['oy'] = -0.02
        #     self._limbs[limb].move_to_neutral()

        # with open(setup_file, 'w') as fp:
        #     for limb in setup:
        #         fp.write('%s\n' % limb)
        #         for param in setup[limb]:
        #             if param == 'pose':
        #                 fp.write('%s ' % param)
        #                 for p in setup[limb][param]:
        #                     fp.write('%s ' % p)
        #                 fp.write('\n')
        #             else:
        #                 fp.write('%s %s\n' % (param, str(setup[limb][param])))

    def load_setup(self, setup_file):
        """ Load the camera calibration data from a file.
        :param setup_file: the file to read the calibration data from
        """
        setup = dict()
        # with open(setup_file, 'r') as fp:
        #     for line in fp:
        #         s = string.split(line)
        #         if len(s) == 1:
        #             limb = s[0]
        #             setup[limb] = dict()
        #         elif len(s) == 2:
        #             if (s[0] == 'distance' or
        #                     s[0] == 'mpp' or
        #                     s[0] == 'ox' or
        #                     s[0] == 'oy'):
        #                 value = float(s[1])
        #             else:
        #                 value = int(s[1])
        #             setup[limb][s[0]] = value
        #         else:
        #             setup[limb][s[0]] = [float(i) for i in s[1:]]
        self._cam_params = setup

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
        # try up to 3 times to find a valid pose
        if not self._move_to_pose(self._top_pose):
            n_tries = self._N_TRIES
            while n_tries > 0:
                print '  trying', n_tries, 'more time(s)'
                n_tries -= 1
                if self._perturbe_pose(self._top_pose):
                    print 'perturbation worked'
                    n_tries = -1
            if not n_tries == -1:
                return False
        return self._try_object()

    def _try_object(self):
        """ Try to select, pick up and place the target object.
        :return: boolean flag on completion
        """
        # record top-down-view image
        imgmsg = self._record_image()
        self._display_image(imgmsg)
        candidates = detect_object_candidates(imgmsg, self._cam_params)
        probabilities = list()
        for candidate in candidates:
            # TODO: add offset to candidate pose
            self._move_to_pose(candidate)
            imgmsg = self._record_image()
            img = select_image_patch(imgmsg, (200, 200))
            # probabilities.append(call_service(img))
        # idx = make_decision(probabilities)
        idx = 0
        self._move_to_pose(candidates[idx])
        if self._grasp_object():
            print '   grasped object'
            # move limb to target location
            self._release_object()
            print '   released object'
            self._limb.move_to_neutral()
            print '  object placed successfully'
            return True
        print '  missed object'
        self._release_object()
        return False

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
        else:
            self._limb.move_to_joint_positions(cmd)
        return True

    def _perturbe_pose(self, pose):
        """ Add a small perturbation to the Cartesian pose of the robot.
        :type pose: [float, float, float, float, float, float]
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
        return self._move_to_pose(p)

    # def _endpoint_pose(self, limb):
    #     qp = self._limbs[limb].endpoint_pose()
    #     r = transformations.euler_from_quaternion(qp['orientation'])
    #     return [qp['position'][0], qp['position'][1], qp['position'][2],
    #             r[0], r[1], r[2]]

    def _record_image(self):
        """ Record an image from one of the robots hand cameras.
        :return: a ROS image message
        """
        s = '/cameras/' + self._arm + '_hand_camera/image'
        self._imgmsg = None
        cam_sub = rospy.Subscriber(s, Image, callback=self._camera_callback)
        while self._imgmsg is None:
            time.sleep(0.1)
        cam_sub.unregister()
        return self._imgmsg

    def _camera_callback(self, data):
        """
        Callback routine for the camera subscriber.
        """
        self._imgmsg = data

    @staticmethod
    def _display_image(imgmsg):
        """ Display an image on the screen of the robot.
        :param imgmsg: a ROS image message
        """
        pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=10,
                              latch=True)
        try:
            pub.publish(resize_imgmsg(imgmsg))
        except TypeError:
            print 'ERROR-display_image-Something is wrong with the ROS image message.'

    def _grasp_object(self):
        """ Close the gripper and validate that it grasped something.
        :return: bool describing if the position move has been preempted by a
        position command exceeding the moving_force threshold denoting a grasp
        """
        self._gripper.close(block=True)
        return self._gripper.gripping()

    def _release_object(self):
        """ Open the gripper.
        """
        self._gripper.open(block=True)

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

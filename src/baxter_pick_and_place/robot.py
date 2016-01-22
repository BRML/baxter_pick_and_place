# Copyright (c) 2015, 2016, BRML
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
import os
import pickle
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
    resize_imgmsg,
    white_imgmsg,
    write_imgmsg,
    segment_bin
)


class Robot(object):

    def __init__(self, limb, outpath):
        """
         A baxter research robot instance with some additional functionality.
         :param limb: The limb to use for the demonstration.
         :param outpath: The path to write output files into.
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
        self._cam_pars = None

        self._outpath = outpath
        self._display_pub = rospy.Publisher('/robot/xdisplay', Image,
                                            queue_size=10, latch=True)

        self._top_pose = [
            0.45,  # x = (+) front, (-) back
            0.0,  # y = (+) left, (-) right
            0.15,  # z = (+) up, (-) down
            -1.0*np.pi,  # roll = horizontal
            0.0*np.pi,  # pitch = vertical
            0.0*np.pi  # yaw = rotation
        ]
        self._bin_pose = None
        self._N_TRIES = 2

        print "\nGetting robot state ... "
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print "Enabling robot... "
        self._rs.enable()

        self._display_image(white_imgmsg())
        self._limb.set_joint_position_speed(0.5)
        self._limb.move_to_neutral()
        self._gripper.calibrate()
        self._camera.resolution = (1280, 800)
        self._camera.fps = 14.0

        self._perform_setup(finger='short')
        self._detect_bin()

    def clean_shutdown(self):
        """ Clean shutdown of the robot.
        :return: True on completion
        """
        print "\nExiting demonstrator ..."
        self._display_image(white_imgmsg())
        self._move_to_pose(self._top_pose)
        self._limb.move_to_neutral()
        if not self._init_state:
            print "Disabling robot..."
            self._rs.disable()
        return True

    """ =======================================================================
        Set system up and prepare things
    ======================================================================= """
    def _perform_setup(self, finger='short'):
        """ Perform the robot limb calibration, i.e., measure the distance from
        the distance sensor to the table and set some other parameters, if no
        setup file exists. If a setup file exists, load it.

        Note: To force performing a new camera setup delete the old setup file.
        :param finger: Fingers used for baxter gripper ['long', 'short'].

        Note: Obviously this cannot replace a full camera calibration and
        proper pose estimation, but for the purpose of this demonstration the
        implemented solution adapted from
          http://sdk.rethinkrobotics.com/wiki/Worked_Example_Visual_Servoing
        is sufficient.
        """
        spath = os.path.join(self._outpath, 'setup')
        if not os.path.exists(spath):
            os.makedirs(spath)
        sfile = os.path.join(spath, 'setup.pkl')

        if not os.path.exists(sfile):
            pose = [0.60, 0.20, 0.0, -1.0*np.pi, 0.0*np.pi, 0.0*np.pi]
            n_calibrations = 10
            self._cam_pars = dict()

            dist = list()
            sensor = baxter_interface.analog_io.AnalogIO(self._arm + '_hand_range')
            print '\nPerforming camera setup ...'
            while len(dist) < n_calibrations:
                p = self._perturbe_pose(pose)
                if self._move_to_pose(p):
                    d = sensor.state()
                    if d < 65000:
                        dist.append(d/1000.0 - p[2])
                        print ' recorded %i/%i measurements (d=%.3fm)' % \
                              (len(dist), n_calibrations, dist[-1])
                    else:
                        print ' ERROR: no valid distance found'
            self._cam_pars['dist'] = np.mean(dist)
            print 'Will be working with average distance d=%.3fm.' % \
                  self._cam_pars['dist']

            self._cam_pars['mpp'] = 0.0025  # meters per pixel @ 1m
            self._cam_pars['x_offset'] = 0.01
            self._cam_pars['y_offset'] = -0.02
            bfh_short = 0.073 + 0.003
            bfh_long = 0.112 + 0.003
            bfh = bfh_short if finger is 'short' else bfh_long
            self._cam_pars['z_offset'] = 0.02 + 0.01 + bfh

            with open(sfile, 'w') as f:
                pickle.dump(self._cam_pars, f)
        else:
            print '\nLoading camera setup ...'
            with open(sfile, 'r') as f:
                self._cam_pars = pickle.load(f)

    def _detect_bin(self):
        """ Detect the bin to put the objects into.
        """
        print '\nLooking for bin to put objects into ...'
        self._move_to_pose(self._top_pose)
        # record top-down-view image
        imgmsg = self._record_image()
        write_imgmsg(imgmsg, os.path.join(self._outpath, 'bin_top_view'))
        rect, corners = segment_bin(imgmsg=imgmsg, outpath=self._outpath,
                                    c_low=50, c_high=190)
        center = rect[0]
        # center = (542, 478)
        print ' Found bin at (%i, %i) pixels.' % (int(center[0]),
                                                  int(center[1]))
        print ' Computing baxter coordinates from pixel coordinates ...'
        self._bin_pose = self._pixel2position(center)
        print 'Detected bin at (%.2f, %.2f, %.2f) m.' % (self._bin_pose[0],
                                                         self._bin_pose[1],
                                                         self._bin_pose[2])

    """ =======================================================================
        Pick and place routine
    ======================================================================= """
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
            idx = np.random.randint(0, 5)
            print "  object '%i' was triggered after %.2fs" % (idx, t)
            return idx

        object_id = _trigger_dummy(10.0)

        # move limb to top-down-view pose
        # try up to 3 times to find a valid pose
        if not self._move_to_pose(self._top_pose):
            n_tries = self._N_TRIES
            while n_tries > 0:
                print '  trying', n_tries, 'more time(s)'
                n_tries -= 1
                ppose = self._perturbe_pose(self._top_pose)
                if self._move_to_pose(ppose):
                    print '  perturbation worked'
                    n_tries = -1
            if not n_tries == -1:
                return False
        return self._try_object(object_id=object_id)

    def _try_object(self, object_id):
        """ Try to select, pick up and place the target object.
        :param object_id: the id of the desired object in the data base.
        :return: boolean flag on completion
        """
        # record top-down-view image
        imgmsg = self._record_image()
        self._display_image(imgmsg)
        # candidates = detect_object_candidates(imgmsg)
        # ignore candidates already in bin
        # candidate = candidates[0]
        # center, dots = candidate
        # pose = self._pixel2position(center)
        # r, p, y = self._estimate_grasp_orientation(dots, object_id)
        # pose[3] = r
        # pose[4] = p
        # pose[5] = y
        pose = [0.54, -0.11, -0.26, 1.0*np.pi, 0.0, 0.0]
        if not self._pick_and_place(pose, object_id=object_id):
            n_tries = self._N_TRIES
            while n_tries > 0:
                print '   trying', n_tries, 'more time(s)'
                n_tries -= 1
                ppose = self._perturbe_pose(pose)
                if self._pick_and_place(ppose, object_id=object_id):
                    n_tries = -1
            if not n_tries == -1:
                print "  failed to grasp object '%i'" % object_id
                return False
        return True

    def _pick_and_place(self, pose, object_id):
        """ Try to approach, grasp, relocate and put down an object.
        :param pose: The pose of the selected object.
        :param object_id: The id of the object in the data base.
        :return: Boolean flag on completion.
        """
        self._approach_pose(pose)
        self._move_to_pose(pose)
        if self._grasp_object():
            print '   grasped object'
            self._move_to_pose(self._top_pose)
            bin_pose = self._perturbe_pose(self._bin_pose)
            self._approach_pose(bin_pose)
            self._move_to_pose(bin_pose)
            self._release_object()
            print '   released object'
            self._move_to_pose(self._top_pose)
            print "  object '%i' placed successfully" % object_id
            return True
        print '  missed object'
        self._release_object()
        return False

    """ =======================================================================
        Helper functions and kinematics
    ======================================================================= """
    def _record_image(self):
        """ Record an image from one of the robots' hand cameras.
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

    def _display_image(self, imgmsg):
        """ Display an image on the screen of the robot.
        :param imgmsg: a ROS image message
        """
        try:
            self._display_pub.publish(resize_imgmsg(imgmsg))
        except TypeError:
            rospy.logerr('display_image - ' +
                         'Something is wrong with the ROS image message.')

    def _pixel2position(self, pixel):
        """ Compute Cartesian position in base coordinates from image pixels.
        Adapted from
          http://sdk.rethinkrobotics.com/wiki/Worked_Example_Visual_Servoing.
        :param pixel: A pixel coordinate
        :return: A pose [x, y, z, 0, 0, 0]
        """
        w, h = self._camera.resolution
        px, py, _, _, _, _ = self._endpoint_pose()
        # x is front/back, so aligned with image height
        x = (pixel[1] - h/2)*self._cam_pars['mpp']*self._cam_pars['dist'] + \
            px + self._cam_pars['x_offset']
        # y is left/right, so aligned with image width
        y = (pixel[0] - w/2)*self._cam_pars['mpp']*self._cam_pars['dist'] + \
            py + self._cam_pars['y_offset']
        z = - self._cam_pars['dist'] + self._cam_pars['z_offset']
        return [x, y, z, -1.0*np.pi, 0., 0.]

    def _grasp_object(self):
        """ Close the gripper and validate that it grasped something.
        :return: bool describing if the position move has been preempted by a
        position command exceeding the moving_force threshold denoting a grasp
        """
        self._gripper.close(block=True)
        return self._gripper.gripping()

    def _release_object(self):
        """ Open the gripper. """
        self._gripper.open(block=True)

    def _move_to_pose(self, pose):
        """ Move robot limb to Cartesian pose.
        :type pose: [float, float, float, float, float, float]
        :param pose: desired Cartesian pose
        :return: boolean flag on completion
        """
        try:
            # TODO: fix this (does not yet work as expected):
            z_min = - self._cam_pars['dist'] + self._cam_pars['z_offset']
            if pose[2] < z_min:
                rospy.logwarn("Replaced requested z=%.2fm with z_min=.2fm." %
                              pose[2], z_min)
                pose[2] = z_min
            cmd = self._inverse_kinematics(pose)
        except Exception:
            return False
        else:
            self._limb.move_to_joint_positions(cmd)
        return True

    def _approach_pose(self, pose):
        """ Move robot limb to Cartesian pose, except for an offset in
        z-direction.
        :type pose: [float, float, float, float, float, float]
        :param pose: desired Cartesian pose to approach
        :return: boolean flag on completion
        """
        # TODO: set approach offset here:
        offset = [0.0, 0.0, 0.05, 0.0, 0.0, 0.0]
        p = [a + b for a, b in zip(pose, offset)]
        return self._move_to_pose(p)

    @staticmethod
    def _perturbe_pose(pose):
        """ Add a small perturbation to the Cartesian pose of the robot.
        :type pose: [float, float, float, float, float, float]
        :param pose: desired Cartesian pose
        :return: perturbed pose
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
        return p

    def _endpoint_pose(self):
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


def pose_to_list(pose):
    """ Baxter pose to pose list.
    :type pose: {'position': (x, y, z), 'orientation': (a, b, c)}
    :param pose: baxter pose
    :return: pose list
    """
    return [pose['position'][0], pose['position'][1], pose['position'][2],
            pose['orientation'][0], pose['orientation'][1], pose['orientation'][2]]

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
import numpy as np

import rospy
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
from motion_planning import SimplePlanner
from utils import list_to_pose_msg, pose_dict_to_list
from utils import pose_dict_to_hom, hom_to_list
import demo.settings as settings
from demo.settings import task_space_limits_m as lims


# Set up logging
_logger = logging.getLogger('baxter')
_logger.setLevel(logging.DEBUG)
_default_loghandler = logging.StreamHandler()
_default_loghandler.setLevel(logging.DEBUG)
_default_loghandler.setFormatter(logging.Formatter('[%(name)s][%(levelname)s] %(message)s'))
_logger.addHandler(_default_loghandler)


def remove_default_loghandler():
    """Call this to mute this library or to prevent duplicate messages
    when adding another log handler to the logger named 'baxter'."""
    _logger.removeHandler(_default_loghandler)


class Baxter(object):
    def __init__(self, sim=False):
        """Hardware abstraction of the Baxter robot using the BaxterSDK
        interface.

        :param sim: Whether in Gazebo (True) or on real Baxter (False).
        """
        self._arms = ['left', 'right']
        self._limbs = {a: baxter_interface.Limb(a)
                       for a in self._arms}
        self._grippers = {a: baxter_interface.Gripper(a)
                          for a in self._arms}
        self._grippers_pars = self._grippers['left'].valid_parameters()
        self._grippers_pars['moving_force'] = 40.0
        self._grippers_pars['holding_force'] = 30.0
        self._sensors = {a: baxter_interface.analog_io.AnalogIO('%s_hand_range' % a)
                         for a in self._arms}
        # Cameras on the Baxter robot are tricky. Due to limited bandwidth
        # only two cameras can be operating at a time.
        # http://sdk.rethinkrobotics.com/wiki/Camera_Control_Tool
        # Default behavior on Baxter startup is for both of the hand cameras
        # to be in operation at a resolution of 320x200 at a frame rate of
        # 25 fps. We get their CameraControllers using the Baxter SDK ...
        self.cameras = {a: baxter_interface.CameraController('%s_hand_camera' % a, sim=sim)
                        for a in self._arms}
        # ... and set their resolution to 1280x800 @ 14 fps.
        for arm in self._arms:
            self.cameras[arm].resolution = (1280, 800)
            self.cameras[arm].fps = 14.0
        # We don't need the CameraControllers any more. Our own module will
        # do the remaining camera handling for us.
        self.cameras = {a: Camera(topic='/cameras/{}_hand_camera/image'.format(a))
                        for a in self._arms}
        self._planner = SimplePlanner()

        self._rs = None
        self._init_state = None
        self.cam_offset = None

        self.z_table = None

    @staticmethod
    def _get_cam_offset():
        """Get the hand_camera--gripper offset in gripper coordinates.
        Note: The offset is the same for the left and right limbs.

        It would be nicer to implement it using a tf.TransformListener between
        topics "/left_gripper" and "/left_hand_camera", but for some reason
        it does not find the topics...
        See http://wiki.ros.org/tf/TfUsingPython#TransformListener for more
        details.

        :return: The offset as a list of length 3 [dx, dy, dz].
        """
        return [0.03828, 0.012, -0.142345]

    def set_up(self):
        """Enable the robot, move both limbs to neutral configuration and
        calibrate both grippers.

        :return:
        """
        _logger.info("Getting robot state.")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        _logger.info("Enabling robot.")
        self._rs.enable()

        _logger.info("Getting camera offset.")
        self.cam_offset = self._get_cam_offset()

        _logger.info("Moving limbs to neutral configuration and calibrate grippers.")
        for arm in self._arms:
            self._limbs[arm].move_to_neutral()
            self._grippers[arm].set_parameters(parameters=self._grippers_pars)
            self._grippers[arm].calibrate()
            # Measured meters per pixel @ 1 m distance
            self.cameras[arm].meters_per_pixel = 0.0025

    def clean_up(self):
        """Open both grippers, move both limbs to neutral configuration and
        disable the robot.

        :return:
        """
        _logger.info("Initiating safe shut-down")
        _logger.info("Moving limbs to neutral configuration")
        for arm in self._arms:
            self._grippers[arm].open()
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
        if isinstance(pose, Pose):
            msg = pose
        else:
            try:
                msg = list_to_pose_msg(pose)
            except ValueError as e:
                _logger.error(str(e))
                raise e
        pose_msg = PoseStamped()
        pose_msg.pose = msg
        pose_msg.header.frame_id = target_frame
        pose_msg.header.stamp = rospy.Time.now()
        return pose_msg

    def endpoint_pose(self, arm):
        """Return the current Cartesian pose of the end effector of the given
        limb.

        :param arm: The arm <'left', 'right'> to control.
        :return: The pose as a list [x, y, z, roll, pitch, yaw].
        """
        return pose_dict_to_list(self._limbs[arm].endpoint_pose())

    @staticmethod
    def sample_task_space_pose(clip_z=False):
        """Sample a random pose from within the robot's task space.
        Note: The orientation is held fixed!

        :param clip_z: Whether to clip the maximum z coordinate.
            Used for calibrating the table height, due to strange behavior of
            the distance sensor.
        :return: The random pose as a list [x, y, z, roll, pitch, yaw].
        """
        if clip_z:
            z_max = 0.0
        else:
            z_max = lims['z_max']
        return [
            (lims['x_max'] - lims['x_min'])*np.random.random_sample() + lims['x_min'],
            (lims['y_max'] - lims['y_min'])*np.random.random_sample() + lims['y_min'],
            (z_max - lims['z_min'])*np.random.random_sample() + lims['z_min'],
            np.pi,
            0.0,
            np.pi
        ]

    def inverse_kinematics(self, arm, pose=None):
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
            s = "No valid configuration found for " \
                "pose {} with {} arm!".format(pose, arm)
            _logger.debug(s)
            raise ValueError(s)

    def ik_either_limb(self, pose):
        """Attempt to solve the inverse kinematics for a given pose with
        either arm. If no solution is found, raise an exception

        :param pose: The pose to stamp. One of
            - None, in which case the current set of joint angles is returned,
            - a ROS Pose,
            - a list of length 6 [x, y, z, roll, pitch, yaw] or
            - a list of length 7 [x, y, z, qx, qy, qz, qw].
        :return: tuple of string and dict:
            - the arm <'left', 'right'> the solution was found for
            - a dictionary of joint name keys to joint angles.
        :raise ValueError: if no valid configuration was found for either arm.
        """
        arm = 'left'
        try:
            cfg = self.inverse_kinematics(arm, pose)
        except ValueError:
            # no valid configuration found for left arm
            arm = 'right'
            try:
                cfg = self.inverse_kinematics(arm, pose)
            except ValueError:
                # no valid configuration found for right arm
                s = "No valid configuration found for pose {} with either arm!".format(pose)
                _logger.warning(s)
                raise ValueError(s)
        return arm, cfg

    def control(self, trajectory):
        """Control one limb using position, velocity or torque control.

        :param trajectory: A generator MotionPlanner instance.
        :return:
        """
        if not isinstance(trajectory, MotionPlanner):
            raise TypeError("'trajectory' must be a MotionPlanner instance!")
        if trajectory.controller_type == 'position':
            for q in trajectory:
                arm = q.keys()[0].split('_')[0]
                self._limbs[arm].move_to_joint_positions(q)
        elif trajectory.controller_type == 'velocity':
            raise NotImplementedError("Need to implement velocity control!")
            # for v in trajectory:
            #     self._limbs[arm].set_joint_velocities(v)
        elif trajectory.controller_type == 'torque':
            raise NotImplementedError("Need to implement torque control!")
            # for t in trajectory:
            #     self._limbs[arm].set_joint_torques(t)
        else:
            raise KeyError("No such control mode: '{}'!".format(trajectory.controller_type))

    def plan(self, target):
        """Plan a trajectory from the current to the target configuration.

        :param target: Dictionary of joint name keys to target joint angles.
        :return: A MotionPlanner trajectory generator.
        """
        arm = target.keys()[0].split('_')[0]
        start = self._limbs[arm].joint_angles()
        self._planner.plan(start=start, end=target)
        return self._planner

    def move_to(self, config=None, pose=None):
        """Shortcut for planning a trajectory to the target configuration
        and executing the trajectory. If a target pose is specified,
        compute the corresponding target configuration using the inverse
        kinematics solver before planning and executing the trajectory.
        Note: Only *one* of <config, pose> can be specified at a time.

        :param config: Dictionary of joint name keys to target joint angles.
        :param pose: The pose to stamp. One of
            - None, in which case the current set of joint angles is returned,
            - a ROS Pose,
            - a list of length 6 [x, y, z, roll, pitch, yaw] or
            - a list of length 7 [x, y, z, qx, qy, qz, qw].
        :return:
        """
        inpt = [False if x is None else True for x in [config, pose]]
        if sum(inpt) == 0 or sum(inpt) == 2:
            raise ValueError("Need to provide either target config or pose!")
        if pose is not None:
            _, config = self.ik_either_limb(pose=pose)
        trajectory = self.plan(target=config)
        self.control(trajectory=trajectory)

    def move_to_neutral(self, arm=None):
        """Move the lift, right or both limbs to their neutral configuration.

        :param arm: The arm <'left', 'right'> to control. If None, move both
            arms.
        :return:
        """
        if arm is None:
            for arm in self._arms:
                self._limbs[arm].move_to_neutral()
        elif arm in self._arms:
            self._limbs[arm].move_to_neutral()
        else:
            raise KeyError("No '{}' limb!".format(arm))

    @staticmethod
    def _gripper_ranges_meters():
        """Grasp ranges for wide and narrow finger slots."""
        return {
            ('narrow', 1): (0.0, 0.018),
            ('narrow', 2): (0.014, 0.037),
            ('narrow', 3): (0.033, 0.056),
            ('narrow', 4): (0.052, 0.075),
            ('wide', 1): (0.071, 0.094),
            ('wide', 2): (0.090, 0.113),
            ('wide', 3): (0.109, 0.132),
            ('wide', 4): (0.128, 0.151)
        }

    def select_gripper_for_object(self, object_id):
        """Select the most suitable gripper for the grasp width corresponding
        to a given object identifier.

        :param object_id: The object identifier.
        :return: The most appropriate limb <'left', 'right'> to grasp the
            requested object.
        """
        dist = 0.0
        min_width = 0.0
        arm = None

        size = settings.object_size_meters[object_id]
        for a in self._arms:
            gr = self._gripper_ranges_meters()[settings.gripper_settings[a]]
            if gr[0] <= size < gr[1]:
                dists = [abs(x - size) for x in gr]
                min_dist = min(dists)
                if min_dist > 0.0:
                    _logger.debug("{} gripper is suitable to grasp object "
                                  "(offset={:.3f} m).".format(
                                      a.capitalize(), min_dist)
                                  )
                    if min_dist > dist:
                        dist = min_dist
                        if gr[0] > min_width:
                            min_width = gr[0]
                        arm = a
                    elif min_dist == dist:
                        # select wider gripper configuration
                        if gr[0] > min_width:
                            min_width = gr[0]
                            arm = a
        if arm is None:
            msg = ("No suitable gripper for object size {:.3f} m installed! "
                   "Check your gripper settings. Currently installed ranges"
                   "are {:.3f}--{:.3f} and {:.3f}--{:.3f}.".format(
                       settings.object_size_meters[object_id],
                       self._gripper_ranges_meters()[settings.gripper_settings['left']][0],
                       self._gripper_ranges_meters()[settings.gripper_settings['left']][1],
                       self._gripper_ranges_meters()[settings.gripper_settings['right']][0],
                       self._gripper_ranges_meters()[settings.gripper_settings['right']][1])
                   )
            _logger.error(msg)
            raise ValueError(msg)
        _logger.info("Selected {} limb to grasp object.".format(arm))
        return arm

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

    def is_gripping(self, arm):
        """Whether the specified gripper is currently holding an object.
        If the currently measured force exceeds half the defined holding
        force, we interpret this as that an object is currently held.

        :param arm: The arm <'left', 'right'> to control.
        :return: Whether an object is held (True) or not (False).
        """
        force_measured = self._grippers[arm].force()
        return force_measured > 0.5*self._grippers_pars['holding_force']

    def release(self, arm):
        """Open the specified gripper. Blocking command.

        :param arm: The arm <'left', 'right'> to control.
        :return:
        """
        return self._grippers[arm].open(block=True)

    def measure_distance(self, arm):
        """Measure the distance from the specified limb to the closest object
        using the limb's infrared sensor.

        :param arm: The arm <'left', 'right'> to control.
        :return: The measured distance in meters or None.
        """
        distance = self._sensors[arm].state()
        if distance < 65000:
            return distance/1000.0
        return None

    def _hom_camera_to_robot(self, arm):
        """Get the homogeneous transformation matrix {}^R\mat{T}_{C} relating
        camera coordinates to robot coordinates.

        :param arm: The arm <'left', 'right'> to control.
        :return: The homogeneous transformation matrix (a 4x4 numpy array).
        """
        ee_pose = self._limbs[arm].endpoint_pose()
        hom_grip_in_rob = pose_dict_to_hom(pose=ee_pose)
        hom_cam_in_grip = np.eye(4)
        hom_cam_in_grip[:-1, -1] = self.cam_offset
        hom_cam_in_rob = np.dot(hom_grip_in_rob, hom_cam_in_grip)
        return hom_cam_in_rob

    def camera_pose(self, arm):
        """Return the current Cartesian pose of the camera of the given limb.

        :param arm: The arm <'left', 'right'> to control.
        :return: The pose as a list [x, y, z, roll, pitch, yaw].
        """
        cam_pose = hom_to_list(matrix=self._hom_camera_to_robot(arm=arm))
        return cam_pose

    def estimate_object_position(self, arm, center):
        """Compute an estimate for the 3D position of an object lying on a
        table with known height.
        Note: This method only works if the gripper is restricted to be
        oriented perpendicular to the table top.

        :param arm: The arm <'left', 'right'> to control.
        :param center: The pixel coordinates to project to robot coordinates.
        :return: The estimated object position as a list of length 3 [x, y, z].
        """
        distance = self.camera_pose(arm=arm)[2] - self.z_table
        cam_coord = self.cameras[arm].projection_pixel_to_camera(pixel=center,
                                                                 z=distance)
        hom_coord = np.asarray(cam_coord + [1])
        rob_coord = np.dot(self._hom_camera_to_robot(arm=arm), hom_coord)
        rob_coord /= rob_coord[-1]
        delta = abs(abs(rob_coord[2]) - abs(self.z_table))
        if delta > 1e-4:
            _logger.warning("Estimated and measured z coordinate of the object "
                            "(table) deviate by {} > 0.0001 mm!".format(delta))
        return [rob_coord[0], rob_coord[1], self.z_table]

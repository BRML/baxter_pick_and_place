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
import os
import random

import cv2
import numpy as np
import rospy

from hardware import img_to_imgmsg
from instruction import client
from settings import settings
from vision import color_difference, draw_detection


class PickAndPlace(object):
    def __init__(self, robot, servo, camera, detection, segmentation, pub_vis, root_dir):
        self._robot = robot
        self._servo = servo
        self._camera = camera
        self._detection = detection
        self._segmentation = segmentation
        self._pub_vis = pub_vis
        self._root = root_dir

        self._logger = logging.getLogger('main.demo')

        self._setup_dir = os.path.join(self._root, 'data', 'setup')
        if not os.path.exists(self._setup_dir):
            self._logger.info('Create setup directory at {}.'.format(self._setup_dir))
            os.makedirs(self._setup_dir)

        # variables for table view calibration
        self._table_image = dict()
        self._table_patches = list()
        self._table_poses = list()

        # safety offset when approaching a pose [x, y, z, r, p, y]
        self._approach_offset = [0, 0, 0.1, 0, 0, 0]

    def publish_vis(self, image):
        """Publish an image to the ROS topic defined in
        settings.debug.topic_img4.

        :param image: The image (numpy array) to publish.
        :return:
        """
        self._pub_vis.publish(img_to_imgmsg(img=image))

    def _wait_for_clear_table(self, arm):
        """Busy wait until the user has cleared the region indicated in the
        published image to be cleared entirely of objects.

        :param arm: The arm <'left', 'right'> to control.
        :return:
        """
        empty = False
        while not empty and not rospy.is_shutdown():
            table_img = self._robot.cameras[arm].collect_image()
            cv2.rectangle(table_img,
                          pt1=settings.table_limits[0],
                          pt2=settings.table_limits[1],
                          color=(0, 0, 255), thickness=3)
            self.publish_vis(image=table_img)
            self._logger.warning("Is the area within the red rectangle devoid of objects?")
            s = raw_input('(yes/no) ')
            if len(s) > 0 and s.lower()[0] == 'y':
                empty = True

    def _move_to_pose_or_raise(self, arm, pose):
        """Shortcut to move the robots' specified limb to the given pose. If
        the pose was not reached, raise an exception.

        :param arm: The arm <'left', 'right'> to control.
        :param pose: The pose to move to. One of
            - a ROS Pose,
            - a list of length 6 [x, y, z, roll, pitch, yaw] or
            - a list of length 7 [x, y, z, qx, qy, qz, qw].
        :return:
        :raise: ValueError if inverse kinematics failed.
        """
        try:
            config = self._robot.ik(arm=arm, pose=pose)
        except ValueError as e:
            self._logger.error("This should not have happened! Abort.")
            raise e
        self._robot.move_to_config(config=config)

    def _move_to_pose_or_dither(self, arm, pose, fix_z=False):
        """Shortcut to move the robots' specified limb to the given pose. If
        the pose was not reached, modify the pose slightly and try again.

        :param arm: The arm <'left', 'right'> to control.
        :param pose: The pose to move to. One of
            - a ROS Pose,
            - a list of length 6 [x, y, z, roll, pitch, yaw] or
            - a list of length 7 [x, y, z, qx, qy, qz, qw].
        :param fix_z: Whether to keep the z coordinate fixed.
        :return: The achieved configuration, a dictionary of joint name keys
            to joint angle values.
        """
        borders = {
            'x_min': pose[0] - 0.025,
            'x_max': pose[0] + 0.025,
            'y_min': pose[1] - 0.025,
            'y_max': pose[1] + 0.025,
            'z_min': pose[2],
            'z_max': pose[2] + (0.025 if not fix_z else 0.0),
            'roll_min': pose[3],
            'roll_max': pose[3],
            'pitch_min': pose[4],
            'pitch_max': pose[4],
            'yaw_min': pose[5] - np.deg2rad(5.0),
            'yaw_max': pose[5] + np.deg2rad(5.0)
        }
        config = None
        while config is None and not rospy.is_shutdown():
            try:
                config = self._robot.ik(arm=arm, pose=pose)
            except ValueError:
                self._logger.debug('Computing IK for pose {} with {} arm '
                                   'failed!'.format(pose, arm))
                pose = self._robot.sample_pose(lim=borders)
        self._robot.move_to_config(config=config)
        return config

    def _calibrate_table_height(self):
        """Calibrate the height of the table in the robot's task coordinates.
        After ensuring that the table has been cleared of objects the robot
        moves its limb to a number of randomly sampled poses in the task
        space. At each pose it measures the distance to the table top using
        an infrared sensor. The estimate of the table height is computed from
        these distance measurements.

        :return: The estimated table height in meters.
        """
        setup_file = os.path.join(self._setup_dir, 'table_height.npz')
        try:
            with np.load(setup_file) as setup:
                height = setup['height']
            self._logger.info('Read table height %.3f m from calibration file.' % height)
        except IOError:
            self._logger.info('Calibrate table height.')
            arm = 'left'
            n_samples = 10
            self._move_to_pose_or_raise(arm=arm, pose=settings.calibration_pose)
            self._wait_for_clear_table(arm=arm)
            heights = list()
            for n in range(n_samples):
                if rospy.is_shutdown():
                    break
                config = None
                while config is None:
                    if rospy.is_shutdown():
                        break
                    random_pose = self._robot.sample_task_space_pose(clip_z=True)
                    try:
                        config = self._robot.ik(arm=arm, pose=random_pose)
                    except ValueError:
                        pass
                self._robot.move_to_config(config=config)
                self.publish_vis(image=self._robot.cameras[arm].collect_image())
                distances = list()
                while len(distances) < 10:
                    d = self._robot.measure_distance(arm=arm)
                    if d is not None:
                        distances.append(d)
                distance = np.mean(distances)
                heights.append(-(distance - self._robot.endpoint_pose(arm=arm)[2]))
            heights = np.asarray(heights)
            h_min = heights.min()
            h_max = heights.max()
            h_mean = heights.mean()
            h_std = heights.std()
            if h_std < 0.005:
                height = h_mean
            else:
                height = h_max
            msg = 'Computed table height to be %.3f m. ' % height
            msg += '(min: {:.3f} m, max: {:.3f} m, mean: {:.3f} m, ' \
                   'std: {:.3f} m).'.format(h_min, h_max, h_mean, h_std)
            self._logger.info(msg)
            np.savez(setup_file, height=height)
            self._robot.move_to_neutral(arm=arm)
            self.publish_vis(image=255*np.ones((800, 1280, 3), dtype=np.uint8))
        return float(height)

    def _calibrate_table_view(self):
        """Calibrate the left- and right hand camera view of the table.
        After ensuring that the table has been cleared of objects the robot
        records reference images of the empty table with the left and right
        hand cameras and selects a number of adjacent patches on the table.

        :return: A dictionary containing
            - the left hand camera reference image (key: 'image_left'),
            - the right hand camera reference image (key: 'image_right'),
            - the selected patches as a (n_patches, 2, 2) numpy array, where
                the first dimension holds the patches and the second dimension
                holds the top left and bottom right points defining the patch,
            - the corresponding 2D positions on the table as a (n_patches, 3)
                numpy array.
        """
        setup_file = os.path.join(self._setup_dir, 'table_view.npz')
        try:
            with np.load(setup_file) as setup:
                data = {
                    'image_left': setup['image_left'],
                    'image_right': setup['image_right'],
                    'patches': setup['patches'],
                    'positions': setup['positions']
                }
            self._logger.info('Read table view from calibration file.')
        except IOError:
            self._logger.info('Calibrate table view.')
            images = dict()
            # define patches
            (xl, yl), (xh, yh) = settings.table_limits
            grid_x = range(xl, xh + 1, 95)
            grid_y = range(yl, yh + 1, 95)
            patches = list()
            centers = list()
            for i in range(len(grid_x) - 1):
                for j in range(len(grid_y) - 1):
                    patches.append(((grid_x[i], grid_y[j]),
                                    (grid_x[i + 1], grid_y[j + 1])))
                    centers.append(((grid_x[i + 1] - grid_x[i])//2 + grid_x[i],
                                    (grid_y[j + 1] - grid_y[j])//2 + grid_y[j]))
            positions = np.empty((len(patches), 3, 2), dtype=np.float32)
            # record images and compute positions corresponding to patches
            for i, arm in enumerate(['left', 'right']):
                if rospy.is_shutdown():
                    break
                self._move_to_pose_or_raise(arm=arm, pose=settings.calibration_pose)
                self._wait_for_clear_table(arm=arm)
                images[arm] = self._robot.cameras[arm].collect_image()
                self.publish_vis(image=images[arm])
                # illustrate patches on the table
                canvas = images[arm].copy()
                for patch, center in zip(patches, centers):
                    cv2.rectangle(canvas, pt1=patch[0], pt2=patch[1],
                                  color=(0, 255, 0), thickness=1)
                    cv2.circle(canvas, center=center, radius=3,
                               color=(0, 255, 0), thickness=1)
                self.publish_vis(image=canvas)

                for j, center in enumerate(centers):
                    if rospy.is_shutdown():
                        break
                    positions[j, :, i] = self._robot.estimate_object_position(arm=arm, center=center)

                self._robot.move_to_neutral(arm=arm)
            data = {
                'image_left': images['left'],
                'image_right': images['right'],
                'patches': np.array(patches),
                'positions': positions.mean(axis=-1)
            }
            max_xyz = positions.std(axis=-1).max(axis=0)
            if max_xyz.max() > 1e-3:
                self._logger.warning("Estimated left and right 2D positions deviate "
                                     "by up to {:.4f} m in x and {:.4f} m in y, "
                                     "which is larger than 0.001 m!".format(
                                         max_xyz[0], max_xyz[1]))
            np.savez(setup_file, **data)
            self.publish_vis(image=255*np.ones((800, 1280, 3), dtype=np.uint8))
        return data

    def _load_external_calibration(self):
        """Load the previously saved external calibration of the demonstration
        setup consisting of the Kinect V2 sensor and the Baxter research robot.

        :return: The affine transformation matrix (a 4x4 numpy array) mapping
            from (Kinect) camera coordinates to robot coordinates.
        """
        setup_file = os.path.join(self._setup_dir, 'external_parameters.npz')
        try:
            with np.load(setup_file) as setup:
                trafo = setup['trafo']
            self._logger.info('Read external camera parameters from calibration file.')
        except IOError:
            self._logger.warning('No external calibration found! Please run the '
                                 'external calibration routine and try again.')
            raise IOError('No external calibration found!')
        return trafo

    def calibrate(self):
        """Perform the calibration of the demonstration setup.

        :return:
        """
        self._logger.info("Perform / read calibration of demonstration setup.")
        # height of the table in robot coordinates
        self._robot.z_table = self._calibrate_table_height()

        # image patches on a table reference image
        # Needed for selecting empty spots on the table for placing objects.
        cfg = self._calibrate_table_view()
        self._table_image = {
            'left': cfg['image_left'],
            'right': cfg['image_right']
        }
        self._table_patches = [(tuple(patch[0]), tuple(patch[1]))
                               for patch in cfg['patches']]
        self._table_poses = [list(pos) + [np.pi, 0.0, np.pi]
                             for pos in cfg['positions']]

        # affine transformation from external camera to Baxter coordinates
        self._camera.trafo = self._load_external_calibration()

    def _get_approach_pose(self, pose):
        """Compute a pose safe for approaching the given pose by adding some
        safety offset.

        :param pose: The pose we wish to approach.
        :return: The approach pose (the original pose + the safety offset).
        """
        return [a + b for a, b in zip(pose, self._approach_offset)]

    def perform(self):
        """Perform the pick-and-place demonstration.

        :return:
        """
        self._logger.info('Starting pick and place demonstration.')
        instr = client.wait_for_instruction()
        while not rospy.is_shutdown() and instr != 'exit':
            obj_id, tgt_id = instr.split(' ')
            self._logger.info('Instructed to take {} and {}.'.format(
                'the {}'.format(obj_id) if obj_id != 'hand' else "'it'",
                'give it to you' if tgt_id == 'hand' else 'put it on the table')
            )

            self._logger.info('Looking for {} and estimate its pose.'.format(obj_id))
            if obj_id == 'hand':
                estimate = self._camera.estimate_hand_position()
                while estimate is None:
                    self._logger.warning("No hand position estimate was found! "
                                         "Please relocate your hand holding the object.")
                    # TODO: adapt this sleep time
                    rospy.sleep(1.0)
                    estimate = self._camera.estimate_hand_position()
                obj_pose = estimate[0] + [np.pi, 0.0, np.pi]
            else:
                img_color, img_depth, _ = self._camera.collect_data(color=True,
                                                                    depth=True,
                                                                    skeleton=False)
                det = self._detection.detect_object(image=img_color,
                                                    object_id=obj_id,
                                                    threshold=0.5)
                draw_detection(image=img_color, detections=det)
                self.publish_vis(image=img_color)
                obj_pose = self._camera.estimate_object_position(img_color=img_color,
                                                                 bbox=det['box'],
                                                                 img_depth=img_depth)
                if obj_pose is None:
                    self._logger.info("I did not find the {}!".format(obj_id))
                    self._logger.info('I resort to searching with the robot.')
                    # TODO: implement looking for the object
                    # move hand camera along pre-defined trajectory over the table
                    # apply object detection until object found or failure
                    # compute 3d coordinates from detection and known height of table
                if obj_pose is None:
                    self._logger.warning("I abort this task! Please start over.")
                    instr = client.wait_for_instruction()
                    continue

                obj_pose += [np.pi, 0.0, np.pi]
            appr_pose = self._get_approach_pose(pose=obj_pose)
            try:
                arm, appr_cfg = self._robot.ik_either_limb(pose=appr_pose)
            except ValueError:
                self._logger.warning("I abort this task! Please start over.")
                instr = client.wait_for_instruction()
                continue

            if tgt_id == 'table':
                self._logger.info('Looking for a spot to put the object down.')
                self._move_to_pose_or_raise(arm=arm, pose=settings.calibration_pose)
                table_img = self._robot.cameras[arm].collect_image()
                idxs = range(len(self._table_patches))
                random.shuffle(idxs)
                tgt_pose = None
                for idx in idxs:
                    if rospy.is_shutdown() or tgt_pose is not None:
                        break
                    (xul, yul), (xlr, ylr) = self._table_patches[idx]
                    table_patch = table_img[yul:ylr, xul: xlr]
                    ref_patch = self._table_image[arm][yul:ylr, xul: xlr]
                    diff, vis_patch = color_difference(image_1=table_patch,
                                                       image_2=ref_patch)
                    self.publish_vis(image=vis_patch)
                    change = diff.mean()*100.0
                    accepted = change <= settings.color_change_threshold
                    self._logger.debug("Patch {} changed by {:.2f}% {} {:.2f}%.".format(
                        idx, change, '<' if accepted else '>', settings.color_change_threshold))
                    if diff.mean()*100.0 < 4.0:
                        tgt_pose = self._table_poses[idx]
                if tgt_pose is None:
                    self._logger.warning("Found no place to put the object down! "
                                         "I abort this task. Please start over.")
                    instr = client.wait_for_instruction()
                    continue

            self._logger.info('Picking up the object.')
            self._logger.info('Attempting to grasp object with {} limb.'.format(arm))
            success = False
            while not success:
                self._robot.move_to_config(config=appr_cfg)
                self._logger.info('Using visual servoing to grasp object.')
                if obj_id == 'hand':
                    self._servo['hand'].servo(arm=arm, object_id=obj_id)
                else:
                    self._servo['table'].servo(arm=arm, object_id=obj_id)
                if self._robot.grasp(arm):
                    success = True
                else:
                    self._logger.info('Something went wrong. I will try again.')
                    self._robot.release(arm)
            self._move_to_pose_or_raise(arm=arm, pose=settings.top_pose)

            self._logger.info('Placing the object.')
            if tgt_id == 'table':
                appr_pose = self._get_approach_pose(pose=tgt_pose)
                appr_cfg = self._move_to_pose_or_dither(arm=arm, pose=appr_pose)
                self._move_to_pose_or_dither(arm=arm, pose=tgt_pose, fix_z=True)
                self._robot.release()
                self._robot.move_to_config(config=appr_cfg)
            else:
                tgt_pose = self._camera.estimate_hand_position()
                while tgt_pose is None:
                    self._logger.warning("No hand position estimate was found! "
                                         "Please relocate your hand.")
                    # TODO: adapt this sleep time
                    rospy.sleep(1.0)
                    tgt_pose = self._camera.estimate_hand_position()
                tgt_pose += [np.pi, 0.0, np.pi]
                self._move_to_pose_or_dither(arm=arm, pose=tgt_pose, fix_z=True)
                self._logger.info('Please take the object from me.')
                while self._robot.is_gripping(arm):
                    rospy.sleep(0.5)
                self._robot.release()
            self._move_to_pose_or_raise(arm=arm, pose=settings.top_pose)
            self._robot.move_to_neutral(arm=arm)
            self._logger.info('I finished my task.')

            instr = client.wait_for_instruction()
        if instr == 'exit':
            self._logger.info('Instructed to exit the demonstration.')
        self._logger.info('Exiting pick and place demonstration.')

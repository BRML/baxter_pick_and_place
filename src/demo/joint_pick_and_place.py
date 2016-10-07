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
import os
import random

import cv2

import rospy

import settings
from hardware import img_to_imgmsg
from instruction import client
from vision import color_difference


# Set up logging
_logger = logging.getLogger('demo')
_logger.setLevel(logging.INFO)
_default_loghandler = logging.StreamHandler()
_default_loghandler.setLevel(logging.INFO)
_default_loghandler.setFormatter(logging.Formatter('[%(name)s][%(levelname)s] %(message)s'))
_logger.addHandler(_default_loghandler)


def remove_default_loghandler():
    """Call this to mute this library or to prevent duplicate messages
    when adding another log handler to the logger named 'demo'."""
    _logger.removeHandler(_default_loghandler)


class PickAndPlace(object):
    def __init__(self, robot, servo, camera, detection, segmentation, pub_vis, root_dir):
        self._robot = robot
        self._servo = servo
        self._camera = camera
        self._detection = detection
        self._segmentation = segmentation
        self._pub_vis = pub_vis
        self._root = root_dir

        self._setup_dir = os.path.join(self._root, 'data', 'setup')
        if not os.path.exists(self._setup_dir):
            _logger.info('Create setup directory at {}.'.format(self._setup_dir))
            os.makedirs(self._setup_dir)

        # safety offset when approaching a pose [x, y, z, r, p, y]
        self._approach_offset = [0, 0, 0.1, 0, 0, 0]

    def _wait_for_clear_table(self, arm):
        """Busy wait until the user has cleared the region indicated in the
        published image to be cleared entirely of objects.

        :param arm: The arm <'left', 'right'> to control.
        :return:
        """
        empty = False
        while not empty and not rospy.is_shutdown():
            table_img = self._robot.cameras[arm].collect_image()
            cv2.rectangle(table_img, pt1=(250, 175), pt2=(1030, 650),
                          color=(0, 0, 255), thickness=3)
            self._pub_vis.publish(img_to_imgmsg(table_img))
            _logger.warning("Is the area within the red rectangle devoid of objects?")
            s = raw_input('(yes/no) ')
            if s.lower()[0] == 'y':
                empty = True

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
            _logger.info('Read table height %.3f m from calibration file.' % height)
        except IOError:
            _logger.info('Calibrate table height.')
            arm = 'left'
            n_samples = 10
            try:
                config = self._robot.inverse_kinematics(arm=arm, pose=settings.calibration_pose)
            except ValueError as e:
                _logger.error("This should not have happened! Abort.")
                raise e
            self._robot.move_to(config=config)
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
                        config = self._robot.inverse_kinematics(arm=arm, pose=random_pose)
                    except ValueError:
                        pass
                self._robot.move_to(config=config)
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
            msg += '(min: %.3f m, max: %.3f m, mean: %.3f m, std: %.3f m).' % (h_min, h_max, h_mean, h_std)
            _logger.info(msg)
            np.savez(setup_file, height=height)
            self._robot.move_to_neutral(arm=arm)
        return height

    def _calibrate_table_view(self):
        # images: two hand camera images <left, right>
        # patches: patches <x, y, w, h> in both images
        setup_file = os.path.join(self._setup_dir, 'table_view.npz')
        try:
            with np.load(setup_file) as setup:
                image_left = setup['image_left']
                image_right = setup['image_right']
                patches = setup['patches']
            _logger.info('Read table view from calibration file.')
        except IOError:
            _logger.info('Calibrate table view.')
            images = dict()
            patches = None
            for arm in ['left', 'right']:
                self._robot.move_to(config=settings.calibration_cfgs[arm])
                self._wait_for_clear_table(arm=arm)
                images[arm] = self._robot.cameras[arm].collect_image()
                # discretize table surface
                #     draw a grid of poses on the table
                #     compute corresponding configs
                #     store corresponding patch of appropriate size
                #         (depends on max object size in meters, mpp and the current distance)
            image_left = images['left']
            image_right = images['right']
            np.savez(setup_file, image_left=images['left'],
                     image_right=images['right'], patches=patches)
        return image_left, image_right, patches

    def _load_external_calibration(self):
        setup_file = os.path.join(self._setup_dir, 'external_parameters.npz')
        try:
            with np.load(setup_file) as setup:
                trafo = setup['trafo']
            _logger.info('Read external camera parameters from calibration file.')
        except IOError:
            _logger.warning('No external calibration found! Please run the '
                            'external calibration routine and try again.')
            raise IOError('No external calibration found!')
        return trafo

    def calibrate(self):
        _logger.info("Perform / read calibration of demonstration setup.")
        # height of the table in robot coordinates
        self._robot.z_table = self._calibrate_table_height()

        # TODO: implement calibration routines
        # image patches corresponding to pre-selected poses / configurations on the
        # table.
        #   - patches: list of quadruples (xul, yul, xlr, ylr)
        #   - poses: list of corresponding poses [x, y, z, roll, pitch, yaw]
        #   - configs: list of corresponding configurations [{'left': {}, 'right':{}}]
        # Needed for selecting empty spots on the table for placing objects.
        # image_left, image_right, patches = self._calibrate_table_view()
        # self._table_image = {'left': None, 'right': None}
        # self._table_patches = []
        # self._table_poses = []
        # self._table_cfgs = []

        # external camera relative to Baxter coordinates
        # trafo = self._load_external_calibration()

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
        _logger.info('Starting pick and place demonstration.')
        instr = client.wait_for_instruction()
        while not rospy.is_shutdown() and instr != 'exit':
            obj_id, tgt_id = instr.split(' ')
            _logger.info('Instructed to take {} and {}.'.format(
                'the {}'.format(obj_id) if obj_id != 'hand' else "'it'",
                'give it to you' if tgt_id == 'hand' else 'put it on the table')
            )

            _logger.info('Looking for {} and estimate its pose.'.format(obj_id))
            if obj_id == 'hand':
                obj_pose = self._camera.estimate_hand_position()
                while obj_pose is None:
                    _logger.warning("No hand position estimate was found! "
                                    "Please relocate your hand holding the object.")
                    # TODO: adapt this sleep time
                    rospy.sleep(1.0)
                    obj_pose = self._camera.estimate_hand_position()
                obj_pose += [0, 0, 0]
            else:
                img_color = self._camera.color.collect_image()
                img_depth = self._camera.depth.collect_image()
                det = self._segmentation.detect_object(image=img_color,
                                                       object_id=obj_id,
                                                       threshold=0.8)
                obj_pose = self._camera.estimate_object_position(img_rgb=img_color,
                                                                 bbox=det['box'],
                                                                 img_depth=img_depth)
                while obj_pose is None:
                    _logger.info("I did not find the {}!".format(obj_id))
                    # TODO: implement looking for the object
                    # move hand camera along pre-defined trajectory over the table
                    # apply object detection until object found or failure
                    # compute 3d coordinates from detection and known height of table
                obj_pose += [0, 0, 0]
            appr_pose = self._get_approach_pose(pose=obj_pose)
            try:
                arm, appr_cfg = self._robot.ik_either_limb(pose=appr_pose)
            except ValueError:
                _logger.warning("I abort this task! Please start over.")
                instr = client.wait_for_instruction()
                continue

            if tgt_id == 'table':
                _logger.info('Looking for a spot to put the object down.')
                self._robot.move_to(config=settings.calibration_cfgs[arm])
                table_img = self._robot.cameras[arm].collect_image()
                idxs = range(len(self._table_patches))
                random.shuffle(idxs)
                tgt_pose = None
                for idx in idxs:
                    xul, yul, xlr, ylr = self._table_patches[idx]
                    table_patch = table_img[yul:ylr, xul: xlr]
                    ref_patch = self._table_image[arm][yul:ylr, xul: xlr]
                    diff, vis_patch = color_difference(image_1=table_patch,
                                                       image_2=ref_patch)
                    self._pub_vis.publish(img_to_imgmsg(vis_patch))
                    # TODO: adapt this threshold
                    if diff.mean()*100.0 < 10.0:
                        tgt_pose = self._table_poses[idx]
                        tgt_cfg = self._table_cfgs[idx][arm]
                        break
                if tgt_pose is None:
                    _logger.warning("Found no place to put the object down! "
                                    "I abort this task. Please start over.")
                    instr = client.wait_for_instruction()
                    continue

            _logger.info('Picking up the object.')
            _logger.info('Attempting to grasp object with {} limb.'.format(arm))
            success = False
            while not success:
                self._robot.move_to(config=appr_cfg)
                _logger.info('Using visual servoing to grasp object.')
                if obj_id == 'hand':
                    self._servo['hand'].servo(arm=arm, object_id=obj_id)
                else:
                    self._servo['table'].servo(arm=arm, object_id=obj_id)
                if self._robot.grasp(arm):
                    success = True
                else:
                    _logger.info('Something went wrong. I will try again.')
                    self._robot.release(arm)
            self._robot.move_to(pose=settings.top_pose)

            _logger.info('Placing the object.')
            if tgt_id == 'table':
                appr_pose = self._get_approach_pose(pose=tgt_pose)
                try:
                    appr_cfg = self._robot.inverse_kinematics(arm=arm, pose=appr_pose)
                except ValueError as e:
                    # TODO: how to handle this case?
                    raise e
                self._robot.move_to(config=appr_cfg)
                self._robot.move_to(config=tgt_cfg)
                self._robot.release()
                self._robot.move_to(config=appr_cfg)
            else:
                tgt_pose = self._camera.estimate_hand_position()
                while tgt_pose is None:
                    _logger.warning("No hand position estimate was found! "
                                    "Please relocate your hand.")
                    # TODO: adapt this sleep time
                    rospy.sleep(1.0)
                    tgt_pose = self._camera.estimate_hand_position()
                tgt_pose += [0, 0, 0]
                try:
                    tgt_cfg = self._robot.inverse_kinematics(arm=arm, pose=tgt_pose)
                except ValueError as e:
                    # TODO: handle this case similar to above
                    raise e
                self._robot.move_to(config=tgt_cfg)
                _logger.info('Please take the object from me.')
                while self._robot.is_gripping(arm):
                    rospy.sleep(0.5)
                self._robot.release()
            self._robot.move_to(pose=settings.top_pose)
            self._robot.move_to_neutral(arm=arm)
            _logger.info('I finished my task.')

            instr = client.wait_for_instruction()
        _logger.info('Exiting pick and place demonstration.')

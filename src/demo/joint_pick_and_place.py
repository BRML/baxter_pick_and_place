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

from instruction import client
from motion_planning import SimplePlanner


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
    def __init__(self, robot, camera, detection, segmentation):
        self._robot = robot
        self._camera = camera
        self._detection = detection
        self._segmentation = segmentation
        self._planner = SimplePlanner()

        # safety offset when approaching a pose [x, y, z, r, p, y]
        self._approach_offset = [0, 0, 0.1, 0, 0, 0]

    def perform(self):
        _logger.info('Starting pick and place demonstration.')
        instr = client.wait_for_instruction()
        while not rospy.is_shutdown() and instr != 'exit':
            obj_id, target_id = instr.split(' ')
            _logger.info('Instructed to take {} and {}.'.format(
                'the {}'.format(obj_id) if obj_id != 'hand' else "'it'",
                'give it to you' if target_id == 'hand' else 'put it on the table')
            )
            _logger.info('Looking for {} and estimate its pose.'.format(obj_id))
            obj_pose = [0.4, 0.5, 0.3, 0, 0, 0]
            if obj_id == 'hand':
                pass
                #   get kinect skeleton
                #   detect hand
                #   if hand not found:
                #       tell user to relocate his hand and try again
                #   else:
                #       compute 3d coordinates from detection skeleton
            else:
                pass
                #   get kinect frame
                #   detect object.
                #   if object not found:
                #       use hand camera to find it
                #   else:
                #         detect object in color image
                #         compute 3d coordinates from color + rectified depth image

            if target_id == 'table':
                _logger.info('Looking for a spot to put the object down.')
                # find an empty spot on the table
                target_pose = [0.5, 0.2, 0.3, 0, 0, 0]

            _logger.info('Picking up the object.')
            approach_pose = [a + b
                             for a, b in zip(obj_pose, self._approach_offset)]
            arm = 'left'
            try:
                approach_cfg = self._robot.inverse_kinematics(arm, approach_pose)
            except ValueError:
                # no valid configuration found for left arm
                arm = 'right'
                try:
                    approach_cfg = self._robot.inverse_kinematics(arm, approach_pose)
                except ValueError:
                    # no valid configuration found for right arm
                    _logger.warning("No valid configuration found " +
                                    "for pose {} ".format(approach_pose) +
                                    "with either arm. Abort this task.")
                    instr = client.wait_for_instruction()
                    continue
            success = False
            while not success:
                trajectory = self._planner.plan(start=None, end=approach_cfg)
                self._robot.control(arm, trajectory)
                _logger.info('Using visual servoing to grasp object.')
                # visual servo to object.
                # If obj_id is hand segmentation needs to return highest score across all classes
                # If target_id is table use known height of table to grasp it
                # If target_id is hand use size of bounding box to estimate distance to object
                if self._robot.grasp(arm):
                    success = True
                else:
                    _logger.info('Something went wrong. I will try again.')
                    self._robot.release(arm)
            # lift object

            if target_id == 'table':
                pass
                # place at pre-computed empty spot using known height of the table
            else:
                pass
                # get kinect skeleton
                # if hand not found tell user to relocate hand and try again
                # compute 3d coordinates of hand
                # move end effector in a straight line toward the approximate coordinates of the hand
                # wait for is_gripping() to return false  ? Will this even work?
                # open gripper
                # retract hand

            instr = client.wait_for_instruction()
        _logger.info('Exiting pick and place demonstration.')

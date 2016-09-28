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

        # safety offset when approaching a pose [x, y, z, r, p, y]
        self._approach_offset = [0, 0, 0.1, 0, 0, 0]

    def calibrate(self):
        # TODO: implement
        # Either perform calibration or load existing calibration file(s)
        _logger.info("Performing calibration/loading calibration file.")

        # height of table
        #  move to calibration_pose
        #  repeat
        #    ask if indicated area under end effector is empty from objects
        #    if not, ask to clear the area from objects
        #    else, proceed
        #  use distance sensor to measure distance to table
        #  compute height of table in robot coordinates and store it
        # mean color of table when empty
        #  record hand camera image and store it

        # size of bounding box in known distance
        #  move to calibration_pose
        #  for each object_id in object_ids do
        #    ask to put object_id into indicated area under end effector
        #    if done, record an image using the hand camera
        #    do object detection and ask if bounding box is sufficiently close
        #    store size of bounding box at distance
        #  repeat for second calibration pose (different distance)
        #  for each object_id store linear equation mapping pixel size to distance
        #   (e.g., assuming longer edge of bb is horizontal)

        # external camera relative to Baxter coordinates
        pass

    def perform(self):
        _logger.info('Starting pick and place demonstration.')
        instr = client.wait_for_instruction()
        while not rospy.is_shutdown() and instr != 'exit':
            obj_id, tgt_id = instr.split(' ')
            _logger.info('Instructed to take {} and {}.'.format(
                'the {}'.format(obj_id) if obj_id != 'hand' else "'it'",
                'give it to you' if tgt_id == 'hand' else 'put it on the table')
            )
            _logger.info('Looking for {} and estimate its pose.'.format(obj_id))
            # TODO: implement
            obj_pose = [0.4, 0.5, 0.3, 0, 0, 0]
            if obj_id == 'hand':
                obj_pose = self._camera.estimate_hand_position()
                while obj_pose is None:
                    _logger.warning("No hand position estimate was found! "
                                    "Please relocate your hand holding the object.")
                    # TODO: adapt this sleep time
                    rospy.sleep(1.0)
                    obj_pose = self._camera.estimate_hand_position() + [0, 0, 0]
            else:
                pass
                #   get kinect frame
                #   detect object.
                #   if object not found:
                #       move hand camera along pre-defined trajectory over the table
                #       apply object detection until object found or failure
                #       compute 3d coordinates from detection and known height of table
                #   else:
                #         detect object in color image
                #         compute 3d coordinates from color + rectified depth image

            if tgt_id == 'table':
                _logger.info('Looking for a spot to put the object down.')
                # move to calibration_pose
                # record an image with the hand camera
                # randomly select patches of appropriate size and compare them to the
                #  corresponding patch in the image stored during calibration
                #  http://jeffkreeftmeijer.com/2011/comparing-images-and-creating-image-diffs/
                # if difference < threshold, patch is assumed to be empty
                target_pose = [0.5, 0.2, 0.3, 0, 0, 0]

            _logger.info('Picking up the object.')
            approach_pose = [a + b
                             for a, b in zip(obj_pose, self._approach_offset)]
            try:
                arm, approach_cfg = self._robot.ik_either_limb(approach_pose)
            except ValueError:
                _logger.warning("I abort this task! Please start over.")
                instr = client.wait_for_instruction()
                continue
            _logger.info('Attempting to grasp object with {} limb.'.format(arm))
            success = False
            while not success:
                self._robot.move_to(config=approach_cfg)
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

            if tgt_id == 'table':
                approach_pose = [a + b
                                 for a, b in zip(target_pose, self._approach_offset)]
                try:
                    approach_cfg = self._robot.inverse_kinematics(arm, approach_pose)
                except ValueError as e:
                    # how to handle this case?
                    raise e
                self._robot.move_to(config=approach_cfg)
                try:
                    cfg = self._robot.inverse_kinematics(arm, target_pose)
                except ValueError as e:
                    # how to handle this case?
                    raise e
                self._robot.move_to(config=cfg)
                self._robot.release()
                self._robot.move_to(config=approach_cfg)
            else:
                pass
                # get kinect skeleton
                # if hand not found tell user to relocate hand and try again
                # compute 3d coordinates of hand
                target_pose = [0.5, 0.2, 0.5, 0, 0, 0]
                try:
                    target_cfg = self._robot.inverse_kinematics(arm, target_pose)
                except ValueError as e:
                    # how to handle this case?
                    raise e
                self._robot.move_to(config=target_cfg)
                _logger.info('Please take the object from me.')
                while self._robot.is_gripping(arm):
                    rospy.sleep(0.5)
                self._robot.release()

                # retract gripper

            instr = client.wait_for_instruction()
        _logger.info('Exiting pick and place demonstration.')

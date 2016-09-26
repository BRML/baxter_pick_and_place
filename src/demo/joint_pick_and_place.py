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

from src.instruction import client


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
        pass

    def perform(self):
        _logger.info('Starting pick and place demonstration.')
        instr = client.wait_for_instruction()
        while not rospy.is_shutdown() and instr != 'exit':
            # TODO: implement pick and place here
            obj_id, target_id = instr.split(' ')
            # if obj_id is an object:
            #   get kinect frame
            #   detect object.
            #   if object not found:
            #       use hand camera to find it
            #   else:
            #         detect object in color image
            #         compute 3d coordinates from color + rectified depth image
            # else if obj_id is 'hand':
            #   get kinect skeleton
            #   detect hand
            #   if hand not found:
            #       tell user to relocate his hand and try again
            #   else:
            #       compute 3d coordinates from detection skeleton
            #
            # if target_id is table, find an empty spot on the table
            #
            # compute config for approach pose. If no success, try other arm.
            # If both fail, abort with error message
            # move arm to approach pose
            # visual servo to object.
            # If target_id is table use known height of table to grasp it
            # If target_id is hand use size of bounding box to estimate distance to object
            # grasp object
            # lift object
            # if target_id is table, place at pre-computed empty spot using known height of the table
            # if target_id is hand,
            #   get kinect skeleton
            #   if hand not found tell user to relocate hand and try again
            #   compute 3d coordinates of hand
            #   move end effector in a straight line toward the approximate coordinates of the hand
            #   wait for is_gripping() to return false  ? Will this even work?
            #   open gripper
            #   retract hand

            instr = client.wait_for_instruction()
        _logger.info('Exiting pick and place demonstration.')

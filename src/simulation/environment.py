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
from numpy import pi
from numpy.random import random_sample
import os

from hardware.utils import list_to_pose_msg
from simulation import (
    load_gazebo_model,
    spawn_gazebo_model,
    delete_gazebo_models
)


# Set up logging
_logger = logging.getLogger('env')
_logger.setLevel(logging.INFO)
_default_loghandler = logging.StreamHandler()
_default_loghandler.setLevel(logging.INFO)
_default_loghandler.setFormatter(logging.Formatter('[%(name)s][%(levelname)s] %(message)s'))
_logger.addHandler(_default_loghandler)


def remove_default_loghandler():
    """Call this to mute this library or to prevent duplicate messages
    when adding another log handler to the logger named 'env'."""
    _logger.removeHandler(_default_loghandler)


class Environment(object):
    def __init__(self, root_dir, object_ids, ws_limits):
        """Handling the virtual demonstration environment in Gazebo.

        :param root_dir: Where the baxter_pick_and_place ROS package resides.
        :param object_ids: The list of object identifiers in the set of
            objects, without the 'background' element. Needs to be
            [object 1, object 2, ..., object N].
        :param ws_limits: The Cartesian space limits in [m] within which the
            objects should be randomly distributed.
        """
        self._ws = os.path.join(root_dir, 'models')
        self._object_ids = object_ids
        self._lim = ws_limits

        self._models = list()

    def _sample_object_pose(self):
        """Randomly sample a pose for an object within specified limits.

        return: A pose as a list [x, y, z, roll, pitch, yaw], where the
            position is specified in [m] and the orientation is given in [rad].
        """
        return [
            (self._lim['x_max'] - self._lim['x_min'])*random_sample() + self._lim['x_min'],
            (self._lim['y_max'] - self._lim['y_min'])*random_sample() + self._lim['y_min'],
            self._lim['z_min'],
            2*pi*random_sample() - pi,
            2*pi*random_sample() - pi,
            2*pi*random_sample() - pi
        ]

    def set_up(self):
        """Place a table in front of Baxter and scatter objects on it."""
        _logger.info("Setting up Gazebo environment.")
        # place table in front of Baxter
        table_urdf = os.path.join(self._ws, 'table', 'model.urdf')
        table_xml = load_gazebo_model(table_urdf)
        table_pose = list_to_pose_msg([0.7, 0, 0, 0, 0, 0])
        spawn_gazebo_model(model_xml=table_xml, model_name='table',
                           robot_namespace='/objects', model_pose=table_pose,
                           model_reference_frame='world')
        self._models.append('table')
        _logger.debug("Successfully spawned table.")

        # randomly scatter objects on table
        for oid in self._object_ids:
            model_urdf = os.path.join(self._ws, oid, 'model.urdf')
            model_xml = load_gazebo_model(model_urdf)
            model_pose = list_to_pose_msg(self._sample_object_pose())
            spawn_gazebo_model(model_xml=model_xml, model_name=oid,
                               robot_namespace='/objects',
                               model_pose=model_pose,
                               model_reference_frame='world')
            self._models.append(oid)
            _logger.debug("Successfully spawned {}.".format(oid))

    def clean_up(self):
        """Delete all spawned Gazebo models."""
        _logger.info("Cleaning up Gazebo environment.")
        delete_gazebo_models(self._models)

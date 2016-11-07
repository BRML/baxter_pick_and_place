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
    delete_gazebo_model,
    delete_gazebo_models
)


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

        self._logger = logging.getLogger('main.env')

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

    def add_model(self, model, pose, ns, frame):
        """Add a model (defined by its URDF description) to Gazebo.

        :param model: The model name.
        :param pose: The pose of the spawned model [x, y, z, r, p, y].
        :param ns: The namespace to spawn the model in.
        :param frame: The reference frame for the model.
        :return:
        """
        model_urdf = os.path.join(self._ws, model, 'model.urdf')
        model_xml = load_gazebo_model(model_urdf)
        spawn_gazebo_model(model_xml=model_xml,
                           model_name=model,
                           robot_namespace=ns,
                           model_pose=list_to_pose_msg(pose),
                           model_reference_frame=frame)
        self._models.append(model)
        self._logger.info("Successfully spawned {}.".format(model))

    def remove_model(self, model):
        """Remove a model from Gazebo (if it exists).

        :param model: The model name.
        :return:
        """
        if model in self._models:
            delete_gazebo_model(model=model)
            self._logger.info("Successfully removed {}.".format(model))

    def set_up(self):
        """Place a table in front of Baxter."""
        self._logger.info("Setting up Gazebo environment.")
        self.add_model(model='table',
                       pose=[0.7, 0, 0, 0, 0, 0],
                       ns='/objects',
                       frame='world')

    def scatter_objects(self):
        """Randomly scatter objects on the table."""
        self._logger.info("Placing objects on table.")
        for oid in self._object_ids:
            self.add_model(model=oid,
                           pose=self._sample_object_pose(),
                           ns='/objects',
                           frame='world')

    def clean_up(self):
        """Delete all spawned Gazebo models."""
        self._logger.info("Cleaning up Gazebo environment.")
        delete_gazebo_models(self._models)

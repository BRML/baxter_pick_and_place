# -*- coding: utf-8 -*-

""" Module for things related to running the baxter pick-and-place
demonstration in the Gazebo physics simulation environment.

"""

from simulation import (
    sim_or_real,
    delete_gazebo_models,
    load_gazebo_model,
    spawn_gazebo_model
)

from environment import (
    Environment,
    remove_default_loghandler as env_remove_default_loghandler
)

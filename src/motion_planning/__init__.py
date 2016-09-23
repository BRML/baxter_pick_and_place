# -*- coding: utf-8 -*-

"""Module for motion planning.

A motion planner computes the trajectory from a given start to end pose or
configuration for either position, velocity or torque control and yields one
step of the trajectory at a time.
That is, the motion planners' plan method should be implemented as a
generator.
"""

from simple import SimplePlanner

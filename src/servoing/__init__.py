# -*- coding: utf-8 -*-

"""Module for implementing visual servoing.

Implements visual servoing with one of Baxter's limbs using the hand camera
for guidance.
"""

from base import remove_default_loghandler as servo_remove_default_loghandler

from distance import ServoingDistance

from size import ServoingSize

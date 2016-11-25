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

import copy
from numpy import pi
import os


# The name or IP of the host PC on which the ELTE Kinect Windows server runs.
elte_kinect_win_host = '10.162.85.173'


# The directory on the Ubuntu machine where the 'py-faster-rcnn' and 'mnc'
# repositories are cloned.
develop_dir = os.path.expanduser('~/software')


# The top pose is the pose taken whenever the robot has completed a task
# (picked up/released an object) or needs to get an overview over the table.
top_pose = [
    0.45,  # x = (+) front, (-) back
    0.0,  # y = (+) left, (-) right
    0.15,  # z = (+) up, (-) down
    pi,  # roll = rotation about x-axis
    0.0,  # pitch = rotation about y-axis
    pi  # yaw = rotation about z-axis
]
# The calibration pose is used for calibrating the robot--table relations.
calibration_pose = [0.45, 0.0, 0.1, pi, 0.0, pi]
# The search pose is used to search for objects if the Kinect failed to
# detect them.
search_pose = [0.75, 0.0, 0.0, pi, 0.0, pi]

# The fingers installed in which slot of the left and right hand of Baxter.
# Needed to select the proper gripper for the object to grasp.
gripper_settings = {
    'left': ('narrow', 3),
    'right': ('narrow', 4)
}

# The measured length of the longer dimension of each object (in the x-y
# plane) in meters.
# Needed for estimating the distance to the object in visual servoing.
object_size_meters = {
    '_person_': 999.99,
    '_bench_': 999.99,
    'bottle': 0.065,
    'cup': 0.086,
    '_apple_': 999.99,
    '_orange_': 999.99,
    'remote': 0.036,
    '_book_': 999.99
}
# The set of objects we know about and Baxter should be able to grasp.
# Needed for the object detection and object segmentation algorithms as well
# as setting up the simulation environment (stripped of the background class)
# if we are working in Gazebo.
# The order needs to be consistent with the order used to train the detection
# and segmentation models!
object_ids = tuple([
    '__background__',
    '_person_',
    '_bench_',
    'bottle',
    'cup',
    '_apple_',
    '_orange_',
    'remote',
    '_book_'
])


# The hand of the human to take objects from or give objects to.
# Can be one of <'right', 'left'>
human_hand = 'right'


# The tolerance for the offset from the segmented objects center to the image
# center in meters.
# Needed for comparing the position error computed in visual servoing.
servo_tolerance_meters = 0.003


# Baxter's hand cameras exposure value (0%--100%)
baxter_cam_exposure = 10


# The threshold for the color change in a table view image patch in percent.
# Needed to detect empty spots on the table.
color_change_threshold = 2.0


# The robot's task space limits in meter
task_space_limits_m = {
    'x_min': 0.35,
    'x_max': 0.84,
    'y_min': -0.475,
    'y_max': 0.475,
    'z_min': -0.19,
    'z_max': 0.33,
}
world_space_limits_m = copy.copy(task_space_limits_m)
world_space_limits_m['z_min'] += 0.92
world_space_limits_m['z_max'] += 0.92

# The robot's task space limits in hand camera pixel coordinates
# (in an image taken with the limb in calibration_pose).
table_limits = ((250, 175), (1030, 650))

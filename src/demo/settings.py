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

from numpy import pi


# TODO: set those poses and configs to something useful
# The top pose is the pose taken whenever the robot has completed a task
# (picked up/released an object) or needs to get an overview over the table.
top_pose = [
    0.45,  # x = (+) front, (-) back
    0.0,  # y = (+) left, (-) right
    0.15,  # z = (+) up, (-) down
    -1.0*pi,  # roll = rotation about x-axis
    0.0*pi,  # pitch = rotation about y-axis
    0.0*pi  # yaw = rotation about z-axis
]
# pre-computed configurations for that pose
top_cfgs = {
    'left': {
        'left_w0': 0.07849614158363094,
        'left_w1': 0.9743649552156033,
        'left_w2': 2.3994108864762955,
        'left_e0': -0.6193236819402792,
        'left_e1': 2.077330658365184,
        'left_s0': -0.8671730199339291,
        'left_s1': -1.4586315923253572
    },
    'right': {
        'right_s0': 0.8603768732034643,
        'right_s1': -1.45721593970738,
        'right_w0': -0.08041448844294827,
        'right_w1': 0.9746724024978123,
        'right_w2': -2.396819412854469,
        'right_e0': 0.6277311779728094,
        'right_e1': 2.076482449080218
    }
}
# The calibration pose is used for calibrating the robot--table relations.
calibration_pose = [0.45, 0.0, -0.1, -1.0*pi, 0.0*pi, 0.0*pi]
calibration_cfgs = {
    'left': {
        'left_w0': -0.7107410074078935,
        'left_w1': 0.4340928648447648,
        'left_w2': -3.059000015258789,
        'left_e0': 0.42645654223310514,
        'left_e1': 2.1246825962582374,
        'left_s0': -1.7016799449920654,
        'left_s1': -0.8455232555031188
    },
    'right': {
        'right_s0': 0.3418348268654378,
        'right_s1': 0.323282667591836,
        'right_w0': 1.2131201858304432,
        'right_w1': -1.5707963705062866,
        'right_w2': 1.6099074175462753,
        'right_e0': 1.4147929189254114,
        'right_e1': 2.029526806713892
    }
}

# TODO: set those ids and values to something useful
# The measured length of the longer dimension of each object (in the x-y
# plane) in meters.
# Needed for estimating the distance to the object in visual servoing.
object_size_meters = {
    'aeroplane': 50,
    'bicycle': 1.5,
    'bird': 0.15,
    'boat': 5,
    'bottle': 0.06,
    'bus': 12,
    'car': 3.8,
    'cat': 0.27,
    'chair': 0.45,
    'cow': 1.6,
    'diningtable': 1.6,
    'dog': 0.6,
    'horse': 2.2,
    'motorbike': 1.8,
    'person': 0.6,
    'pottedplant': 0.3,
    'sheep': 0.9,
    'sofa': 2.5,
    'train': 12,
    'tvmonitor': 1.2
}
# The set of objects we know about and Baxter should be able to grasp.
# Needed for the object detection and object segmentation algorithms as well
# as setting up the simulation environment (stripped of the background class)
# if we are working in Gazebo.
object_ids = tuple(['__background__'] + sorted(object_size_meters.keys()))

# The tolerance for the offset from the segmented objects center to the image
# center in meters.
# Needed for comparing the position error computed in visual servoing.
servo_tolerance_meters = 0.005

# The topic on which debugging image output is visualized
topic_visualization = '/visualization/image'

# TODO: set these limits to something useful
workspace_limits_m = {
    'x_min': 0.3,
    'x_max': 0.7,
    'y_min': -0.4,
    'y_max': 0.4,
    'z_min': 0.75,
    'z_max': 1.15
}

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
top_pose = [
    0.45,  # x = (+) front, (-) back
    0.0,  # y = (+) left, (-) right
    0.15,  # z = (+) up, (-) down
    -1.0*pi,  # roll = rotation about x-axis
    0.0*pi,  # pitch = rotation about y-axis
    0.0*pi  # yaw = rotation about z-axis
]
top_cfgs = {  # pre-computed configurations for that pose
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

calibration_poses = (
    [0.45, 0.0, 0.15, -1.0*pi, 0.0*pi, 0.0*pi],
    [0.45, 0.0, -0.1, -1.0*pi, 0.0*pi, 0.0*pi]
)
calibration_cfgs = (
    {
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
    },
    {
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
)

# TODO: set those ids to something useful
object_ids = (
    '__background__',
    'aeroplane',
    'bicycle',
    'bird',
    'boat',
    'bottle',
    'bus',
    'car',
    'cat',
    'chair',
    'cow',
    'diningtable',
    'dog',
    'horse',
    'motorbike',
    'person',
    'pottedplant',
    'sheep',
    'sofa',
    'train',
    'tvmonitor'
)

# TODO: set these limits to something useful
workspace_limits_m = {
    'x_min': 0.3,
    'x_max': 0.7,
    'y_min': -0.4,
    'y_max': 0.4,
    'z_min': 0.75,
    'z_max': 1.15
}

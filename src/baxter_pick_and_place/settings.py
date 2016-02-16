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

import numpy as np


""" object list """
object_list = [
    'duplo_brick',
    # 'extra_mints',
    # 'glue_stick',
    # 'golf_ball',
    # 'robot'
]

""" top pose """
top_pose = [
    0.45,  # x = (+) front, (-) back
    0.0,  # y = (+) left, (-) right
    0.15,  # z = (+) up, (-) down
    -1.0*np.pi,  # roll = rotation about x-axis
    0.0*np.pi,  # pitch = rotation about y-axis
    0.0*np.pi  # yaw = rotation about z-axis
]

""" setup pose """
setup_pose = [0.60, 0.20, 0.0, -1.0*np.pi, 0.0*np.pi, 0.0*np.pi]

""" table workspace in pixel coordinates (background images / top pose) """
parameters = dict()
parameters['x_min'] = 400
parameters['x_max'] = 1020
parameters['y_min'] = 295
parameters['y_max'] = 670

""" blob center tolerance in [m] for viusal servoing """
vs_tolerance = 0.005

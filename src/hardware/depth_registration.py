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

# DISCLAIMER:
# When implementing a C++ node using the kinect2_registration library from
# the iai_kinect2 package we found that the depth image is cropped before
# being scaled and the field of view of the resulting color and depth images
# are quite different, such that they cannot easily be superimposed. We thus
# resort to a very naive workaround by scaling and shifting the depth image
# by constant factors to match the color image reasonable well.
# This is feasible since all we need is a rough estimate for the depth, the
# visual servoing will take care of the rest.
# It would be much nicer to use the iai_kinect2 or WindowsSDK libraries,
# though.

import numpy as np

import cv2


def register_depth(img_depth, size_color):
    """Compute a depth image that matches the color image.
    Note: This method implements a naive workaround to 'proper' depth image
    registration, as for instance implemented in the iai_kinect2 package.

    :param img_depth: A depth image.
    :param size_color: The size of the color image.
    :return: A depth image matching the color image.
    """
    if size_color == (540, 960):
        # magic factors for matching the depth image to the 960x540 color image
        scale_factor = 1.45
        x_shift = 120
        y_shift = -38
    elif size_color == (1080, 1920):
        # magic factors for matching the depth image to the 1920x1080 color image
        # TODO: modify magic factors for full HD resolution
        scale_factor = 1.45*2
        x_shift = 120*2
        y_shift = -38*2
    else:
        raise ValueError("Not defined for size {}!".format(size_color))
    dsize = tuple([int(x * scale_factor)
                   for x in img_depth.shape[:2]][::-1])
    img_depth = cv2.resize(img_depth, dsize, interpolation=cv2.INTER_CUBIC)
    output = np.zeros(size_color, dtype=img_depth.dtype)
    h0, w0 = output.shape[:2]
    h2, w2 = img_depth.shape[:2]
    # where to put image
    x0 = min(max(x_shift, 0), w0)
    y0 = min(max(y_shift, 0), h0)
    x1 = min(x_shift + w2, w0)
    y1 = min(y_shift + h2, h0)
    # what to put
    x2 = min(-x_shift if x_shift < 0 else 0, w2)
    y2 = min(-y_shift if y_shift < 0 else 0, h2)
    x3 = min(x2 + x1 - x0, w2)
    y3 = min(y2 + y1 - y0, h2)
    output[y0:y1, x0:x1] = img_depth[y2:y3, x2:x3]
    return output


def get_depth(img_depth, size_color, pixel_color):
    """Compute the depth at a given pixel in the RGB image from a corresponding
    depth image.

    :param img_depth: A depth image.
    :param size_color: The size of the color image.
    :param pixel_color: The requested pixel in the color image.
    :return: The depth at the requested pixel.
    """
    if size_color == (540, 960):
        # magic factors for matching the depth image to the 960x540 color image
        scale_factor = 1.45
        x_shift = 120
        y_shift = -38
    elif size_color == (1080, 1920):
        # magic factors for matching the depth image to the 1920x1080 color image
        # TODO: modify magic factors for full HD resolution
        scale_factor = 1.45*2
        x_shift = 120*2
        y_shift = -38*2
    else:
        raise ValueError("Not defined for size {}!".format(size_color))
    cx, cy = pixel_color
    scaled_x = int((cx - x_shift)/scale_factor)
    scaled_y = int((cy - y_shift)/scale_factor)
    return img_depth[scaled_y, scaled_x]/1000.0


def blend(img_color, img_depth):
    """Blend the color and registered depth image on top of each other.

    :param img_color: A color image.
    :param img_depth: A corresponding depth image.
    :return: The registered depth image overlaid on top of the color image.
    """
    img_reg = register_depth(img_depth, img_color.shape[:2])
    img_reg = cv2.cvtColor(cv2.convertScaleAbs(img_reg), cv2.COLOR_GRAY2BGR)
    alpha = 0.3
    beta = 1.0 - alpha
    gamma = 0.0
    return cv2.addWeighted(img_color, alpha, img_reg, beta, gamma)

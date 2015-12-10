# Copyright (c) 2015, BRML
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

import cv_bridge
import numpy as np


def detect_object_candidates(imgmsg):
    """ Detect object candidates and return their poses in robot coordinates.
    :param imgmsg: a ROS image message
    :return: list of poses (6-tuples) of object candidates
    """
    image = _imgmsg2img(imgmsg)
    cam_params = None
    locations = _find_object_candidates(image)
    poses = [_pose_from_location(location, cam_params) for location in locations]
    return [(0., 0., 0., 0., 0., 0.), (0., 0., 0., 0., 0., 0.)]


def select_image_patch(imgmsg, patch_size=(200, 200)):
    """ Select an image patch of size patch_size around an object.
    :param imgmsg: a ROS image message
    :param patch_size: the patch size of the image patch to return
    :return: an image patch
    """
    image = _imgmsg2img(imgmsg)
    center = _find_object_candidates(image, 1)[0]
    cx, cy = center
    mask = [cx - patch_size[0]/2, cy - patch_size[1]/2,
            cx + patch_size[0]/2, cy + patch_size[1]/2]
    return None  # image[mask]


def _imgmsg2img(imgmsg):
    """ Convert a ROS image message to a numpy array holding the image.
    :param imgmsg: a ROS image message
    :return: a numpy array containing an RGB image
    """
    try:
        img = cv_bridge.CvBridge().imgmsg_to_cv2(imgmsg, 'rgb8')
    except cv_bridge.CvBridgeError, e:
        print e
    return np.asarray(img)


def _find_object_candidates(image, n_candidates=None):
    """ Detect object candidates in an image and return their pixel
    coordinates.
    :param image: a camera image
    :param n_candidates: the number of candidates to return
    :return: a list of object candidate locations in pixel coordinates
    """
    locations = [(0, 0), (0, 0)]
    if n_candidates is not None and n_candidates <= len(locations):
        return locations[:n_candidates]
    return locations


def _pose_from_location(location, cam_params):
    """ Compute robot coordinates from pixel coordinates.
    :param location: a pixel coordinate
    :param cam_params: camera parameters
    :return: an object pose (6-tuple)
    """
    return 0., 0., 0., 0., 0., 0.

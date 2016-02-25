# Copyright (c) 2015--2016, BRML
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
import cv2
import numpy as np


""" ===========================================================================
    Image and ROS image message handling
=========================================================================== """


def resize_imgmsg(imgmsg):
    """ Resize a ROS image message to fit the screen of the baxter robot.
    :param imgmsg: a ROS image message of arbitrary image size
    :return: a ROS image message containing an image with 1024x600 pixels
    """
    img = imgmsg2img(imgmsg)
    img = cv2.resize(img, (1024, 600))
    return img2imgmsg(img)


def white_imgmsg():
    """ A white image of size 1024x600 pixels fitting the screen of the baxter
    robot.
    :return: a ROS image message.
    """
    img = np.ones((600, 1024, 1), dtype=np.uint8)
    img *= 255
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img2imgmsg(img)


def black_imgmsg():
    """ A black image of size 1024x600 pixels fitting the screen of the baxter
    robot.
    :return: a ROS image message.
    """
    img = np.zeros((600, 1024, 1), dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img2imgmsg(img)


def cut_imgmsg(imgmsg, x_min, y_min, x_max, y_max):
    """ Cut an image to a given region of interest.
    :param imgmsg: a ROS image message
    :param x_min: ROI left upper corner x coordinate
    :param y_min: ROI left upper corner y coordinate
    :param x_max: ROI right lower corner x coordinate
    :param y_max: ROI right lower corner y coordinate
    :return: a ROS image message
    """
    img = imgmsg2img(imgmsg)
    img = img[y_min:y_max, x_min:x_max]
    return img2imgmsg(img)


def write_imgmsg(imgmsg, filename):
    """ Convert a ROS image message into an image and save it to the disc.
    :param imgmsg: a ROS image message
    :param filename: the filename to save the image, without the extension
    """
    img = imgmsg2img(imgmsg)
    cv2.imwrite(filename + '.jpg', img)


def write_img(img, filename):
    """ Save an image to the disc.
    :param img: a numpy array containing an image
    :param filename: the filename to save the image, without the extension
    """
    cv2.imwrite(filename + '.jpg', img)


def img2imgmsg(img):
    """ Convert a numpy array holding an image to a ROS image message.
    :param img: a numpy array
    :return: a ROS image message
    """
    try:
        imgmsg = cv_bridge.CvBridge().cv2_to_imgmsg(img, 'bgr8')
    except cv_bridge.CvBridgeError:
        raise
    return imgmsg


def imgmsg2img(imgmsg):
    """ Convert a ROS image message to a numpy array holding the image.
    :param imgmsg: a ROS image message
    :return: a numpy array containing an RGB image
    """
    try:
        img = cv_bridge.CvBridge().imgmsg_to_cv2(imgmsg, 'bgr8')
    except cv_bridge.CvBridgeError:
        raise
    except AttributeError:
        print 'ERROR-imgmsg2img-Something is wrong with the ROS image message.'
        raise
    return img


def mask_imgmsg_region(imgmsg, corners):
    """ Mask the region of a rotated rectangle (defined by its corner pixels).
    :param imgmsg: a ROS image message
    :param corners: the corners of a rotated rectangle region
    :return: a ROS image message
    """
    img = imgmsg2img(imgmsg)
    corners = np.asarray(corners, dtype=np.int32)
    cv2.fillConvexPoly(img, points=corners, color=(0, 0, 0))
    return img2imgmsg(img)


def string2imgmsg(s1, s2=None):
    """ Convert up to two strings into a ROS image message.
    :param s1: the first line of text
    :param s2: the (optional) second line of text
    :return: a ROS image message
    """
    white = (255, 255, 255)
    canvas = imgmsg2img(black_imgmsg())
    h, w = canvas.shape[:2]
    font_face = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 3.0
    thickness = 7

    (w1, h1), _ = cv2.getTextSize(text=s1,
                                  fontFace=font_face, fontScale=font_scale,
                                  thickness=thickness)
    org = ((w-w1)/2, (h/3)+(h1/2))
    cv2.putText(canvas, text=s1, org=org,
                fontFace=font_face, fontScale=font_scale,
                color=white, thickness=thickness)
    if s2:
        (w1, h1), _ = cv2.getTextSize(text=s2,
                                      fontFace=font_face, fontScale=font_scale,
                                      thickness=thickness)
        org = ((w-w1)/2, (2*h/3) + (h1/2))
        cv2.putText(canvas, text=s2, org=org,
                    fontFace=font_face, fontScale=font_scale,
                    color=white, thickness=thickness)
    return img2imgmsg(canvas)


""" ===========================================================================
    Object detection and pose estimation
=========================================================================== """


def detect_object_candidates(imgmsg, cam_params):
    """ Detect object candidates and return their poses in robot coordinates.
    :param imgmsg: a ROS image message
    :param cam_params: dict of camera parameters
    :return: list of poses (6-tuples) of object candidates
    """
    image = imgmsg2img(imgmsg)
    locations = _find_object_candidates(image)
    poses = [_pose_from_location(location, cam_params) for location in locations]
    return [(0., 0., 0., 0., 0., 0.), (0., 0., 0., 0., 0., 0.)]


def select_image_patch(imgmsg, patch_size=(200, 200)):
    """ Select an image patch of size patch_size around an object.
    :param imgmsg: a ROS image message
    :param patch_size: the patch size of the image patch to return
    :return: an image patch
    """
    image = imgmsg2img(imgmsg)
    center = _find_object_candidates(image, 1)[0]
    cx, cy = center
    mask = [cx - patch_size[0]/2, cy - patch_size[1]/2,
            cx + patch_size[0]/2, cy + patch_size[1]/2]
    return None  # image[mask]


def pose_estimation(imgmsg, obj_id):
    img = imgmsg2img(imgmsg)
    table = _segment_table(img, verbose=True)
    candidates = _find_object_candidates(table)
    for candidate in candidates:
        # check if it is the object we are looking for
        if True:
            # find point correspondences  # TODO: how????
            # rvecs, tvecs, inliers = cv2.solvePnPRansac(object_points, image_points, cam_params['mtx'], cam_params['dist'])
            # compensate for camera offset
            # compensate for finger offset
            # compensate current pose when recording the image
            # move to pose
            # grasp object
            pass


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


def _pose_from_location(object_points, image_points, cam_params):
    """ Compute robot base coordinates from image coordinates.
    :param object_points: list of object points from find_calibration_pattern
    :param image_points: list of image points from find_calibration_pattern
    :param cam_params: camera parameters
    :return: a relative object pose (6-tuple)
    """

    return 0., 0., 0., 0., 0., 0.

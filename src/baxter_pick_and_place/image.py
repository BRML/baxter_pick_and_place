# Copyright (c) 2015, 2016, BRML
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
import os


""" ===========================================================================
    Image and ROS image message handling
=========================================================================== """


def resize_imgmsg(imgmsg):
    """ Resize a ROS image message to fit the screen of the baxter robot.
    :param imgmsg: a ROS image message of arbitrary image size
    :return: a ROS image message containing an image with 1024x600 pixels
    """
    img = _imgmsg2img(imgmsg)
    img = cv2.resize(img, (1024, 600))
    return _img2imgmsg(img)


def white_imgmsg():
    """ A white image of size 1024x600 pixels fitting the screen of the baxter
    robot.
    :return: a ROS image message.
    """
    img = np.ones((600, 1024, 1), dtype=np.uint8)
    img *= 255
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return _img2imgmsg(img)


def black_imgmsg():
    """ A black image of size 1024x600 pixels fitting the screen of the baxter
    robot.
    :return: a ROS image message.
    """
    img = np.zeros((600, 1024, 1), dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return _img2imgmsg(img)


def write_imgmsg(imgmsg, filename):
    """ Convert a ROS image message into an image and save it to the disc.
    :param imgmsg: a ROS image message
    :param filename: the filename to save the image, without the extension
    """
    img = _imgmsg2img(imgmsg)
    cv2.imwrite(filename + '.jpg', img)


def _write_img(img, filename):
    """ Save an image to the disc.
    :param img: a numpy array containing an image
    :param filename: the filename to save the image, without the extension
    """
    cv2.imwrite(filename + '.jpg', img)


def _img2imgmsg(img):
    """ Convert a numpy array holding an image to a ROS image message.
    :param img: a numpy array
    :return: a ROS image message
    """
    try:
        imgmsg = cv_bridge.CvBridge().cv2_to_imgmsg(img, 'bgr8')
    except cv_bridge.CvBridgeError:
        raise
    return imgmsg


def _imgmsg2img(imgmsg):
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


""" ===========================================================================
    Object detection and pose estimation
=========================================================================== """


def detect_object_candidates(imgmsg, cam_params):
    """ Detect object candidates and return their poses in robot coordinates.
    :param imgmsg: a ROS image message
    :param cam_params: dict of camera parameters
    :return: list of poses (6-tuples) of object candidates
    """
    image = _imgmsg2img(imgmsg)
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


def pose_estimation(imgmsg, obj_id):
    img = _imgmsg2img(imgmsg)
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


""" ===========================================================================
    Machine vision
=========================================================================== """


def _segment_table(img, lower_hsv=np.array([38, 20, 125]),
                   upper_hsv=np.array([48, 41, 250]), verbose=False):
    """ Segment table in input image based on color information.
    :param img: the image to work on
    :param lower_hsv: lower bound of HSV values to segment
    :param upper_hsv: upper bound of HSV values to segment
    :param verbose: show intermediate images or not
    :return: image cropped to ROI (table)
    """
    cv2.imshow('Image', img)
    cv2.waitKey(3)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    if verbose:
        cv2.imshow('Mask', mask)
        cv2.waitKey(3)
        table = cv2.bitwise_and(img, img, mask=mask)
        cv2.imshow('Table', table)
        cv2.waitKey(3)
        kernel = np.ones((5, 5), np.uint8)
        closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,
                                   iterations=10)
        cv2.imshow('Closing', closing)
        cv2.waitKey(3)
        table2 = cv2.bitwise_and(img, img, mask=closing)
        cv2.imshow('Table 2', table2)
        cv2.waitKey(3)

    points = cv2.findNonZero(mask)
    try:
        x, y, w, h = cv2.boundingRect(points)
        if verbose:
            image = img.copy()
            cv2.rectangle(image, (x, y), (x + w, y + h),
                          np.array([0, 255, 0]), 2)
            cv2.imshow('Rectangle', image)
            cv2.waitKey(3)
            roi = image[y:y + h, x:x + w]
            cv2.imshow('ROI', roi)
            cv2.waitKey(3)
    except:
        raise
    return img[y:y + h, x:x + w]


def segment_bin(imgmsg, outpath=None, c_low=50, c_high=270):
    """ Segment bin to put objects into.
    :param imgmsg: a ROS image message
    :param outpath: the path to where to write intermediate images to
    :param c_low: lower bound for Canny threshold
    :param c_high: upper bound for Canny threshold
    :return: ((cx, cy), (w, h), alpha), corners
    """
    img = _imgmsg2img(imgmsg)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # do Canny edge extraction and widen edges
    canny = cv2.Canny(gray, c_low, c_high, apertureSize=3)
    kernel = np.ones((3, 3), np.uint8)
    canny = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel, iterations=1)
    if outpath:
        _write_img(canny, os.path.join(outpath, 'canny'))

    # flood fill image to extract bounded regions
    print " Flood fill edge image. This may take a while! Please be patient."
    flooded = _flood_fill(canny)
    if outpath:
        _write_img(flooded, os.path.join(outpath, 'flooded'))

    # find contours, largest contour equals our bin
    contours, _ = cv2.findContours(canny, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    ret = list()
    for c in contours:
        if 10000 < cv2.contourArea(c) < 100000:
            rect = cv2.minAreaRect(c)
            box = cv2.cv.BoxPoints(rect)
            ret.append((rect, box))
            b = np.int0(box)
            cv2.drawContours(img, [b], 0, (0, 255, 0), 2)
            cv2.circle(img, (int(rect[0][0]), int(rect[0][1])), 3, (0, 0, 255), 2)
    if outpath:
        _write_img(img, os.path.join(outpath, 'contours'))
    if len(ret) != 1:
        raise Exception('ERROR-segment_bin-No/Too many contour(s) found!' +
                        ' Please adjust Canny thresholds.')
    return ret


def _flood_fill(canny):
    """ Flood fill an image from the border to leave only connected components
    untouched. Adapted from
      http://sdk.rethinkrobotics.com/wiki/Worked_Example_Visual_Servoing
    :param canny: Edge image to work on.
    :return: Image with connected components black, remainder white.
    """
    h, w = canny.shape
    white = 255
    black = 0

    # set border pixels to white
    canny[0, :] = white
    canny[-1, :] = white
    canny[:, 0] = white
    canny[:, -1] = white

    # prime to-do list
    to_do = [(2, 2), (2, h - 3), (w - 3, h - 3), (w - 3, 2)]

    # test pixels
    while len(to_do) > 0:
        x, y = to_do.pop()
        if canny[y, x] == black:
            canny[y, x] = white
            to_do.append((x, y - 1))
            to_do.append((x, y + 1))
            to_do.append((x - 1, y))
            to_do.append((x + 1, y))
    return canny

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
import matplotlib.pyplot as plt
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
    img = imgmsg2img(imgmsg)
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
    return _img2imgmsg(img)


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


""" ===========================================================================
    Machine vision
=========================================================================== """


def segment_area(imgmsg, outpath=None, th=200, c_low=50, c_high=270,
                 ff_connectivity=4, a_low=100, a_high=200):
    """ Segment connected components on an image based on area using
    machine vision algorithms of increasing complexity.
    :param imgmsg: a ROS image message
    :param outpath: the path to where to write intermediate images to
    :param th: threshold for binary thresholding operation
    :param c_low: lower Canny threshold
    :param c_high: upper Canny threshold
    :param ff_connectivity: neighborhood relation (4, 8) to use for flood fill
    operation
    :param a_low: lower bound for contour area
    :param a_high: upper bound for contour area
    :returns: a tuple (rroi, roi) containing the rotated roi and corners and
    the upright roi enclosing the connected component
    """
    img = imgmsg2img(imgmsg)
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    equ = cv2.equalizeHist(gray)

    # try binary threshold
    _, thresh = cv2.threshold(equ, th, 255, cv2.THRESH_BINARY)
    contour = _extract_contour(thresh, c_low, c_high, a_low, a_high)
    if contour is not None:
        array = thresh
        title = 'threshold'
    else:  # try opening to remove small regions
        kernel = np.ones((2, 2), np.uint8)
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel,
                                   iterations=1)
        contour = _extract_contour(opening, c_low, c_high,
                                   a_low, a_high)
        if contour is not None:
            array = opening
            title = 'opening'
        else:  # try outline of segmented regions
            kernel = np.ones((2, 2), np.uint8)
            closing = cv2.morphologyEx(opening, cv2.MORPH_OPEN,
                                       kernel, iterations=1)
            outline = cv2.morphologyEx(closing, cv2.MORPH_GRADIENT,
                                       kernel, iterations=2)
            contour = _extract_contour(outline, c_low, c_high,
                                       a_low, a_high)
            if contour is not None:
                array = outline
                title = 'outline'
            else:  # see if flood-filling the image helps
                h, w = outline.shape[:2]
                mask = np.zeros((h+2, w+2), np.uint8)
                mask[1:-1, 1:-1] = outline
                seed_pt = (mask.shape[0], 0)
                flooded = gray.copy()
                flags = ff_connectivity | cv2.FLOODFILL_FIXED_RANGE
                cv2.floodFill(flooded, mask, seed_pt, (255, 255, 255),
                              (50,)*3, (255,)*3, flags)
                contour = _extract_contour(flooded, c_low, c_high,
                                           a_low, a_high)
                if contour is not None:
                    array = flooded
                    title = 'flooded'
    if contour is None:
        raise ValueError('No contour found!')

    # rotated roi
    rrect = cv2.minAreaRect(contour)
    box = cv2.cv.BoxPoints(rrect)
    # upright roi
    x, y, w, h = cv2.boundingRect(contour)

    if outpath:
        plt.figure(figsize=(11, 20))
        plt.subplot(121)
        plt.imshow(array, cmap='gray')
        plt.title(title)

        sample = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # rotated roi
        b = np.int0(box)
        cv2.drawContours(sample, [b], 0, (0, 255, 0), 2)
        cv2.circle(sample, (int(rrect[0][0]), int(rrect[0][1])), 4,
                   (0, 255, 0), 2)
        # upright roi
        cv2.rectangle(sample, (x, y), (x+w, y+h), (255, 0, 0), 2)
        cv2.circle(sample, (x+w/2, y+h/2), 3, (255, 0, 0), 2)
        plt.subplot(122)
        plt.imshow(sample)

        plt.savefig(os.path.join(outpath, 'bin_contours.jpg'),
                    bbox_inches='tight')

    return (rrect, box), (x, y, w, h)


def _extract_contour(img, c_low=50, c_high=270, a_low=100, a_high=200):
    """ Apply Canny edge detection to an image and return maximal
    contour found.
    :param img: the image to work on
    :param c_low: lower Canny threshold
    :param c_high: upper Canny threshold
    :param a_low: lower bound for contour area
    :param a_high: upper bound for contour area
    :returns: the found contour, or None
    """
    canny = cv2.Canny(img, c_low, c_high, apertureSize=3)
    kernel = np.ones((3, 3), np.uint8)
    canny = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel,
                             iterations=1)

    contours, _ = cv2.findContours(canny, cv2.RETR_LIST,
                                   cv2.CHAIN_APPROX_SIMPLE)
    max_area = 0.0
    contour = None
    for c in contours:
        area = cv2.contourArea(c)
        if a_low < area < a_high:
            if area > max_area:
                max_area = area
                contour = c
    return contour

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
import cv2
import numpy as np


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


def _imgmsg2img(imgmsg):
    """ Convert a ROS image message to a numpy array holding the image.
    :param imgmsg: a ROS image message
    :return: a numpy array containing an RGB image
    """
    try:
        img = cv_bridge.CvBridge().imgmsg_to_cv2(imgmsg, 'rgb8')
    except cv_bridge.CvBridgeError:
        raise
    except AttributeError:
        print 'ERROR-imgmsg2img-Something is wrong with the ROS image message.'
    return np.asarray(img)


def resize_imgmsg(imgmsg):
    """ Resize a ROS image message to fit the screen of the baxter robot.
    :param imgmsg: a ROS image message of arbitrary image size
    :return: a ROS image message containing an image with 1024x600 pixels
    """
    img = _imgmsg2img(imgmsg)
    img = cv2.resize(img, (1024, 600))
    try:
        imgmsg = cv_bridge.CvBridge().cv2_to_imgmsg(img, 'rgb8')
    except cv_bridge.CvBridgeError:
        raise
    return imgmsg


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


def _segment_table(img, lower_hsv=np.array([38, 20, 125]),
                   upper_hsv=np.array([48, 41, 250]), verbose=False):
    """ Segment table in input image based on color information.
    :param img: the image to work on
    :param lower_hsv: lower bound of HSV values to segment
    :param upper_hsv: upper bound of HSV values to segment
    :param verbose: show intermediate images or not
    :return: image cropped to ROI (table)
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    if verbose:
        cv2.imshow('Mask', mask)
        table = cv2.bitwise_and(img, img, mask=mask)
        cv2.imshow('Table', table)
        kernel = np.ones((5, 5), np.uint8)
        closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=10)
        cv2.imshow('Closing', closing)
        table2 = cv2.bitwise_and(img, img, mask=closing)
        cv2.imshow('Table 2', table2)

    points = cv2.findNonZero(mask)
    x, y, w, h = cv2.boundingRect(points)
    if verbose:
        image = img.copy()
        cv2.rectangle(image, (x, y), (x + w, y + h), np.array([0, 255, 0]), 2)
        cv2.imshow('Rectangle', image)
        roi = image[y:y + h, x:x + w]
        cv2.imshow('ROI', roi)
    return img[y:y + h, x:x + w]


def _pose_from_location(location, cam_params):
    """ Compute robot coordinates (relative to wrist pose) from pixel
    coordinates.
    :param location: a pixel coordinate
    :param cam_params: camera parameters
    :return: a relative object pose (6-tuple)
    """
    return 0., 0., 0., 0., 0., 0.


def find_calibration_pattern(imgmsg, imgname, verbose=False):
    """ Find 9x6 chessboard calibration pattern in an image and return point
    correspondences ([mm] to [pixel]) if successful.
    :param imgmsg: a ROS image message
    :param imgname: filename to write the image to if pattern was found,
    without the extension
    :param verbose: show control images or not
    :return: return value, object points, image points
    """
    """ Define calibration pattern """
    pattern_size = (9, 6)
    objp = np.zeros((np.prod(pattern_size), 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= 25.17  # mm
    """ Define sub-pixel criteria """
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    img = _imgmsg2img(imgmsg)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, pattern_size)
    if ret:
        cv2.cornerSubPix(gray, corners, winSize=(5, 5), zeroZone=(-1, -1),
                         criteria=criteria)
        cv2.imwrite(imgname + '.jpg', img)
        if verbose:
            cv2.drawChessboardCorners(img, pattern_size, corners, ret)
            cv2.imwrite(imgname + 'dcc.jpg', img)
        return ret, objp, corners
    print "Did not find calibration pattern."
    return ret, None, None


def calibrate_camera(object_points, image_points, test_imgmsg, test_imgname):
    """ Perform camera calibration.
    :param object_points: list of object points from find_calibration_pattern
    :param image_points: list of image points from find_calibration_pattern
    :param test_imgmsg: ROS image message to test the calibration on
    :param test_imgname: filename to write the test image to
    :return: re-projection error, camera matrix, distortion coefficients,
    rotation vectors, translation vectors
    """
    test_image = _imgmsg2img(test_imgmsg)
    h, w = test_image.shape[:2]
    re_err, camera_matrix, dist_coeffs, rvecs, tvecs = \
        cv2.calibrateCamera(object_points, image_points, (w, h))
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix,
                                                           dist_coeffs,
                                                           (w, h), 1, (w, h))
    undst = cv2.undistort(test_image, camera_matrix, dist_coeffs,
                          newCameraMatrix=new_camera_matrix)
    x, y, w, h = roi
    undst = undst[y:y + h, x:x + w]
    cv2.imwrite(test_imgname, undst)
    return re_err, camera_matrix, dist_coeffs, rvecs, tvecs

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

import cv2


# Set colors for visualisation of detections. Note BGR convention used!
red = (0, 0, 255)
green = (0, 255, 0)
blue = (255, 0, 0)
yellow = (0, 255, 255)
black = (0, 0, 0)
white = (255, 255, 255)


def textbox(image, text, org, font_face, font_scale, thickness, color, color_box):
    """Draw a filled box with text placed on top of it.

    :param image: The image to draw on.
    :param text: The text to put there.
    :param org: The origin of the text (bottom left corner) (x, y).
    :param font_face: The font to use.
    :param font_scale: The scale for the font.
    :param thickness: The thickness of the font.
    :param color: The color of the font (BGR).
    :param color_box: The color of the box (BGR).
    :return:
    """
    (w, h), _ = cv2.getTextSize(text=text,
                                fontFace=font_face, fontScale=font_scale,
                                thickness=thickness)
    ox, oy = [int(o) for o in org]
    cv2.rectangle(image, pt1=(ox - 2, oy + 2), pt2=(ox + 2 + w, oy - 2 - h),
                  color=color_box, thickness=cv2.cv.CV_FILLED)
    cv2.putText(image, text=text, org=(ox, oy),
                fontFace=font_face, fontScale=font_scale,
                thickness=thickness, color=color)


def draw_detection(image, detections):
    """Draw all given detections onto the given image and label them with
    their object identifier and score.
    Note: Modifies the passed image!

    :param image: An image (numpy array) of shape (height, width, 3).
    :param detections: A (list of) dictionary(ies) of detection containing
            'id': The object identifier.
            'score: The score of the detection (scalar).
            'box': The bounding box of the detection; a (4,) numpy array.
           ['mask': The segmentation of the detection; a (height, width,)
                numpy array.]
    :return:
    """
    if not isinstance(detections, list):
        detections = [detections]
    for detection in detections:
        if 'mask' in detection and detection['mask'] is not None:
            image[detection['mask'] == 255] = yellow
        if detection['box'] is not None:
            oid, s, b = [detection['id'], detection['score'], detection['box']]
            cv2.rectangle(image, pt1=(b[0], b[1]), pt2=(b[2], b[3]),
                          color=red, thickness=2)
            textbox(image, text='%s %.3f' % (oid, s), org=(b[0] + 3, b[3] - 3),
                    font_face=cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale=0.5, thickness=2, color=black, color_box=white)


def mask_to_rroi(mask):
    """Compute the minimum enclosing rectangle (possibly rotated) for a given
    pre-computed segmentation of an object.

    :param mask: The binary mask image to compute the minimum enclosing
        rectangle (possibly rotated) for.
    :return: The minimum enclosing rectangle, defined by
        <(cx, cy), (w, h), alpha>, where w and h are the width and height of
        the rectangle centered around cx and cy and rotated by alpha degrees.
    """
    contours, _ = cv2.findContours(image=mask, mode=cv2.RETR_LIST,
                                   method=cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 1:
        raise ValueError("Expected to find exactly one contour!")
    # TODO: why does alpha not always point along the longer axis?
    return cv2.minAreaRect(contours[0])


def draw_rroi(image, rroi):
    """Draw the given rotated rectangle onto the given image.
    Note: Modifies the passed image!

    :param image: An image (numpy array) of shape (height, width, 3).
    :param rroi: The rotated rectangle ((cx, cy), (w, h), angle) to draw.
    :return:
    """
    h, w = image.shape[:2]
    # visualize image center (=target)
    cv2.circle(image, center=(w//2, h//2), radius=4, color=blue, thickness=2)
    # visualize rroi center and orientation
    center = tuple(np.round(rroi[0]).astype(np.uint32))
    cv2.circle(image, center=center, radius=4, color=green, thickness=2)
    angle = np.deg2rad(rroi[2])
    line_length = 20
    pt2 = [a + b
           for a, b in zip(rroi[0], (line_length*np.cos(angle),
                                     line_length*np.sin(angle)))]
    pt2 = tuple(np.round(pt2).astype(np.uint32))
    cv2.line(image, pt1=center, pt2=pt2, color=green, thickness=1)
    # draw rotated rectangle
    box = np.int0(cv2.cv.BoxPoints(rroi))
    cv2.drawContours(image, contours=[box], contourIdx=0, color=green,
                     thickness=2)


def color_difference(image_1, image_2):
    """Compute the pixel-wise delta E color difference for two RGB images.
    Note: We compare RGB images, which means that the result is not as
        accurate as if we converted the images to LAB color space before
        computing the delta E difference. This way the result is nicely
        interpretable, though.
    See
        http://jeffkreeftmeijer.com/2011/comparing-images-and-creating-image-diffs/
    and
        https://www.cis.rit.edu/~cnspci/media/software/deltae.py
    for more information.

    :param image_1: A color image ((h, w, 3) uint8 numpy array).
    :param image_2: A color image ((h, w, 3) uint8 numpy array).
    :return: The delta E color difference as a (h, w) float numpy array with
        values in the range [0, 1] and
        the the corresponding gray scale image.
    """
    assert image_1.shape == image_2.shape
    # Scale both color images to unit range
    image_1, image_2 = [img.astype(np.float32)/255.
                        for img in [image_1, image_2]]
    # compute the delta E color difference per pixel
    delta_e = np.sqrt(np.sum((image_2 - image_1)**2, axis=-1)/3.)
    return delta_e, np.asarray(255.*delta_e, dtype=np.uint8)

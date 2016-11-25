#!/usr/bin/env python

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

import logging
import numpy as np

import cv2


class ObjectSegmentation(object):
    def __init__(self, root_dir, object_ids):
        self._classes = object_ids

        self._logger = logging.getLogger('main.opencv')

    def init_model(self, warmup=False):
        """This method is here for compatibility reasons."""
        pass

    def _morphology(self, image):
        """Using a number of image morphology operators to find the single
        connected region in the image.
        Note: This method will fail as soon as there are multiple objects in
        the given image patch (i.e., objects on the table ly to close to
        each other.

        :param image: A binary input image of shape (height, width).
        :return: A triple containing
            - a list of contours,
            - the bounding box, or None and
            - the segmentation mask, or None.
        """
        kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3, 3))
        opening = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        contour, _ = cv2.findContours(opening, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        # canvas = np.zeros_like(image, np.uint8)
        # for cnt in contour:
        #     cv2.drawContours(canvas, [cnt], 0, np.random.randint(1, 256), -1)
        if len(contour) == 1:
            mask = np.zeros_like(image, np.uint8)
            cnt = contour[0]
            cv2.drawContours(mask, [cnt], 0, 255, -1)

            x, y, w, h = cv2.boundingRect(contour[0])
            box = np.array([x, y, x + w, y + h])
            # self._logger.debug("Bounding box of segmentation is {}.".format(
            #     np.array_str(box, precision=2, suppress_small=True)))
        elif len(contour) == 0:
            self._logger.warning("No contour found!")
            box = None
            mask = None
        else:
            # self._logger.debug("Multiple contours found! Dilate image and repeat.")
            img = np.zeros_like(image, np.uint8)
            for cnt in contour:
                cv2.drawContours(img, [cnt], 0, 255, -1)
            kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(5, 5))
            dilated = cv2.morphologyEx(img, cv2.MORPH_DILATE, kernel)
            return self._morphology(image=dilated)
        return contour, box, mask

    def segment(self, image):
        """Segment a single object in the given image.

        :param image: An image (numpy array) of shape (height, width, 3).
        :return: A tuple containing
            'box': The bounding box of the object; a (4,) numpy array.
            'mask': The segmentation of the object; a (height, width) numpy
                array.
        """
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        equ = cv2.equalizeHist(gray)
        _, thresh = cv2.threshold(equ, thresh=200, maxval=255, type=cv2.THRESH_BINARY)
        contour, box, mask = self._morphology(image=thresh)
        return box, mask

    def detect_object(self, image, object_id, threshold=0.5):
        """This method is here for compatibility reasons.

        :param image: An image (numpy array) of shape (height, width, 3).
        :param object_id: One object identifier string contained in the list
            of objects.
        :param threshold: The threshold (0, 1) on the score for a detection
            to be considered as valid.
        :return: A dictionary containing the detection with
            'id': The object identifier.
            'score: The score of the detection (scalar).
            'box': The bounding box of the detection; a (4,) numpy array.
            'mask': The segmentation of the detection; a (height, width)
                numpy array.
        """
        if object_id not in self._classes:
            raise KeyError("Object {} is not contained in the defined "
                           "set of objects!".format(object_id))
        box, mask = self.segment(image=image)

        return {'id': object_id, 'score': 0.0, 'box': box, 'mask': mask}

    def detect_best(self, image, threshold=0.5):
        """This method is here for compatibility reasons.

        :param image: An image (numpy array) of shape (height, width, 3).
        :param threshold: The threshold (0, 1) on the score for a detection
            to be considered as valid.
        :return: A dictionary containing the detection with
            'id': The object identifier.
            'score: The score of the detection (scalar).
            'box': The bounding box of the detection; a (4,) numpy array.
            'mask': The segmentation of the detection; a (height, width)
                numpy array.
        """
        box, mask = self.segment(image=image)

        return {'id': 'some object', 'score': 0.0, 'box': box, 'mask': mask}


if __name__ == '__main__':
    import os
    from visualization_utils import draw_detection
    path = '/home/mludersdorfer/software/ws_baxter_pnp/src/baxter_pick_and_place'
    print 'started'
    classes = ('__background__',
               'aeroplane', 'bicycle', 'bird', 'boat',
               'bottle', 'bus', 'car', 'cat', 'chair',
               'cow', 'diningtable', 'dog', 'horse',
               'motorbike', 'person', 'pottedplant',
               'sheep', 'sofa', 'train', 'tvmonitor')
    od = ObjectSegmentation(root_dir=path, object_ids=classes)
    od.init_model(warmup=False)

    for img_file in [os.path.join(path, 'data', '%s.jpg' % i)
                     for i in ['remote']]:
        img = cv2.imread(img_file)
        if img is not None:
            det = od.detect_best(img)
            # det = od.detect_object(img, 'person', 0.8)
            if det['box'] is not None:
                draw_detection(img, det)
                cv2.imshow('image', img)
                cv2.imshow('mask', det['mask'])
                cv2.waitKey(0)
    cv2.destroyAllWindows()
    print 'ended'

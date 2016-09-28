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
import os
import time

import cv2

from visualization_utils import (
    red, black, white,
    textbox
)

from init_paths import set_up_faster_rcnn
set_up_faster_rcnn()
# suppress caffe logging up to 0 debug, 1 info 2 warning 3 error
os.environ['GLOG_minloglevel'] = '2'
import caffe as caffe_frcnn
from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect


# Use RPN for proposals
cfg.TEST.HAS_RPN = True

# Set up logging
_logger = logging.getLogger('frcnn')
_logger.setLevel(logging.INFO)
_default_loghandler = logging.StreamHandler()
_default_loghandler.setLevel(logging.INFO)
_default_loghandler.setFormatter(logging.Formatter('[%(name)s][%(levelname)s] %(message)s'))
_logger.addHandler(_default_loghandler)


def remove_default_loghandler():
    """Call this to mute this library or to prevent duplicate messages
    when adding another log handler to the logger named 'frcnn'."""
    _logger.removeHandler(_default_loghandler)


class ObjectDetection(object):
    def __init__(self, root_dir, object_ids):
        """Instantiates a 'faster R-CNN' object detector object.

        :param root_dir: Where the baxter_pick_and_place ROS package resides.
        :param object_ids: The list of object identifiers in the set of
            objects. Needs to be
            [background, object 1, object 2, ..., object N].
        """
        self._classes = object_ids
        self._net = None
        self._prototxt = os.path.join(root_dir, 'models', 'VGG16',
                                      'faster_rcnn_test.pt')
        self._caffemodel = os.path.join(root_dir, 'data', 'VGG16',
                                        'VGG16_faster_rcnn_final.caffemodel')
        if not os.path.isfile(self._prototxt):
            raise RuntimeError("No network architecture specification found "
                               "at %s!" % self._prototxt)
        if not os.path.isfile(self._caffemodel):
            raise RuntimeError("No network parameter dump found at %s!" %
                               self._caffemodel)

    def init_model(self, warmup=False):
        """Load the pre-trained Caffe model onto GPU0.

        :param warmup: Whether to warm up the model on some dummy images.
        :return:
        """
        caffe_frcnn.set_mode_gpu()
        gpu_id = 0
        caffe_frcnn.set_device(gpu_id)
        cfg.GPU_ID = gpu_id

        self._net = caffe_frcnn.Net(self._prototxt, self._caffemodel, caffe_frcnn.TEST)
        _logger.info('Loaded network %s.' % self._caffemodel)
        if warmup:
            dummy = 128 * np.ones((300, 500, 3), dtype=np.uint8)
            for _ in xrange(2):
                _, _ = im_detect(self._net, dummy)

    def detect(self, image):
        """Feed forward the given image through the previously loaded network.
        Return scores and bounding boxes for all abject proposals and classes.

        :param image: An image (numpy array) of shape (height, width, 3).
        :return: A tuple of two numpy arrays, the n_proposals x n_classes
            scores and the corresponding n_proposals x 4*n_classes bounding
            boxes, where each bounding box is defined as <xul, yul, xlr, ylr>.
        """
        if self._net is None:
            raise RuntimeError("No loaded network found! "
                               "Did you run init_model()?")
        if len(image.shape) != 3 and image.shape[2] != 3:
            raise ValueError("Image must be a three channel color image "
                             "with shape (h, w, 3)!")
        start = time.time()
        scores, boxes = im_detect(self._net, image)
        _logger.info('Detection took {:.3f}s for {:d} object proposals'.format(
            time.time() - start, boxes.shape[0])
        )
        return scores, boxes

    def detect_object(self, image, object_id, threshold=0.5):
        """Feed forward the given image through the previously loaded network.
        Return the bounding box with the highest score for the requested
        object class.

        :param image: An image (numpy array) of shape (height, width, 3).
        :param object_id: One object identifier string contained in the list
            of objects.
        :param threshold: The threshold (0, 1) on the score for a detection
            to be considered as valid.
        :return: The best score and bounding box if the best score > threshold,
            The best score and None otherwise.
        """
        if object_id not in self._classes:
            raise KeyError("Object {} is not contained in the defined "
                           "set of objects!".format(object_id))
        scores, boxes = self.detect(image=image)

        # Find scores for requested object class
        cls_idx = self._classes.index(object_id)
        cls_scores = scores[:, cls_idx]
        cls_boxes = boxes[:, 4*cls_idx:4*(cls_idx + 1)]

        best_idx = np.argmax(cls_scores)
        best_score = cls_scores[best_idx]
        best_box = cls_boxes[best_idx]
        _logger.info('Best score for {} is {:.3f} {} {:.3f}'.format(
            object_id,
            best_score,
            '>=' if best_score > threshold else '<',
            threshold)
        )
        if best_score > threshold:
            return best_score, best_box
        return best_score, None

    def detect_best(self, image, threshold=0.5):
        """Feed forward the given image through the previously loaded network.
        Return the bounding box with the highest score amongst all classes.

        :param image: An image (numpy array) of shape (height, width, 3).
        :param threshold: The threshold (0, 1) on the score for a detection
            to be considered as valid.
        :return: The object id with the best score and bounding box if the
            best score > threshold, he object id, best score and None
            otherwise.
        """
        scores, boxes = self.detect(image=image)

        # find best score among all classes (except background)
        best_proposal, best_class = np.unravel_index(scores[:, 1:].argmax(),
                                                     scores[:, 1:].shape)
        best_class += 1  # compensate for background
        best_score = scores[best_proposal, best_class]
        best_box = boxes[best_proposal, 4*best_class:4*(best_class + 1)]
        best_object = self._classes[best_class]

        _logger.info('Best score for {} is {:.3f} {} {:.3f}'.format(
            best_object,
            best_score,
            '>=' if best_score > threshold else '<',
            threshold)
        )
        if best_score > threshold:
            return best_object, best_score, best_box
        return best_object, best_score, None

    @staticmethod
    def draw_detection(image, object_ids, scores, boxes):
        """Draw all given bounding boxes onto the given image and label them
        with their object identifier and score.
        Note: Modifies the passed image!

        :param image: An image (numpy array) of shape (height, width, 3).
        :param object_ids: One object identifier string contained in the list
            of objects.
        :param scores: Detection scores (n x 1 numpy array).
        :param boxes: Detected bounding boxes (n x 4 numpy array).
        :return:
        """
        if boxes is None:
            return None
        if len(boxes.shape) == 1:
            scores = scores[np.newaxis]
            boxes = boxes[np.newaxis, :]
        if isinstance(object_ids, str):
            object_ids = [object_ids]*len(scores)
        if len(scores) != len(boxes):
            raise ValueError("Require scores and corresponding bounding boxes!")
        for oid, s, b in zip(object_ids, scores, boxes):
            cv2.rectangle(image, pt1=(b[0], b[1]), pt2=(b[2], b[3]),
                          color=red, thickness=2)
            textbox(image, text='%s %.3f' % (oid, s), org=(b[0] + 3, b[3] - 3),
                    font_face=cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale=0.5, thickness=2, color=black, color_box=white)


if __name__ == '__main__':
    path = '/home/mludersdorfer/software/ws_baxter_pnp/src/baxter_pick_and_place'
    _logger.info('started')
    classes = ('__background__',
               'aeroplane', 'bicycle', 'bird', 'boat',
               'bottle', 'bus', 'car', 'cat', 'chair',
               'cow', 'diningtable', 'dog', 'horse',
               'motorbike', 'person', 'pottedplant',
               'sheep', 'sofa', 'train', 'tvmonitor')
    od = ObjectDetection(root_dir=path, object_ids=classes)
    od.init_model(warmup=True)

    for img_file in [os.path.join(path, 'data', '%s.jpg' % i)
                     for i in ['004545', '000456', '000542', '001150', '001763']]:
        img = cv2.imread(img_file)
        if img is not None:
            oid, score, box = od.detect_best(img, 0.8)
    #         score, box = od.detect_object(img, 'dog', 0.8)
            if box is not None:
                # od.draw_detection(img, 'dog', score, box)
                od.draw_detection(img, oid, score, box)
                cv2.imshow('image', img)
                cv2.waitKey(0)
    cv2.destroyAllWindows()
    _logger.info('ended')

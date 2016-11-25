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

from init_paths import set_up_rfcn
set_up_rfcn()
# suppress caffe logging up to 0 debug, 1 info 2 warning 3 error
os.environ['GLOG_minloglevel'] = '2'
import caffe
from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect
from fast_rcnn.nms_wrapper import nms


# Use RPN for proposals
cfg.TEST.HAS_RPN = True


class ObjectDetection(object):
    def __init__(self, root_dir, object_ids):
        """Instantiates a 'R-FCN' object detector object.

        :param root_dir: Where the baxter_pick_and_place ROS package resides.
        :param object_ids: The list of object identifiers in the set of
            objects. Needs to be
            [background, object 1, object 2, ..., object N].
        """
        self._classes = object_ids

        self._logger = logging.getLogger('main.rfcn')

        self._net = None
        self._prototxt = os.path.join(root_dir, 'models', 'ResNet-101',
                                      'rfcn_test.pt')
        self._caffemodel = os.path.join(root_dir, 'data', 'ResNet-101',
                                        'rfcn_ohem_490000.caffemodel')
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
        caffe.set_mode_gpu()
        gpu_id = 0
        caffe.set_device(gpu_id)
        cfg.GPU_ID = gpu_id

        self._net = caffe.Net(self._prototxt, self._caffemodel, caffe.TEST)
        self._logger.info('Loaded network %s.' % self._caffemodel)
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
        self._logger.debug('Detection took {:.3f}s for {:d} object proposals'.format(
            time.time() - start, boxes.shape[0])
        )

        # perform non-maximum suppression
        for cls_idx, cls in enumerate(self._classes):
            dets = np.hstack((boxes[:, 4:8],
                              scores[:, cls_idx][:, np.newaxis])).astype(np.float32)
            keep = nms(dets, 0.3)
            mask = np.zeros_like(scores, dtype=np.bool)
            mask[:, cls_idx] = True
            mask[keep, cls_idx] = False
            scores[mask] = 0.0
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
        :return: A dictionary containing the detection with
            'id': The object identifier.
            'score: The score of the detection (scalar).
            'box': The bounding box of the detection; a (4,) numpy array.
        """
        if object_id not in self._classes:
            raise KeyError("Object {} is not contained in the defined "
                           "set of objects!".format(object_id))
        scores, boxes = self.detect(image=image)

        # Find scores for requested object class
        cls_idx = self._classes.index(object_id)
        cls_scores = scores[:, cls_idx]

        best_idx = np.argmax(cls_scores)
        best_score = cls_scores[best_idx]
        best_box = boxes[best_idx, 4:8]

        self._logger.debug('Best score for {} is {:.3f} {} {:.3f}'.format(
            object_id,
            best_score,
            '>=' if best_score > threshold else '<',
            threshold)
        )
        if best_score > threshold:
            return {'id': object_id, 'score': best_score, 'box': best_box}
        return {'id': object_id, 'score': best_score, 'box': None}

    def detect_best(self, image, threshold=0.5):
        """Feed forward the given image through the previously loaded network.
        Return the bounding box with the highest score amongst all classes.

        :param image: An image (numpy array) of shape (height, width, 3).
        :param threshold: The threshold (0, 1) on the score for a detection
            to be considered as valid.
        :return: A dictionary containing the detection with
            'id': The object identifier.
            'score: The score of the detection (scalar).
            'box': The bounding box of the detection; a (4,) numpy array.
        """
        scores, boxes = self.detect(image=image)

        # get rid of scores for unwanted classes
        for idx, object_id in enumerate(self._classes):
            if object_id.startswith('_'):
                scores[:, idx] = 0.0

        # find best score among all classes (except background)
        best_proposal, best_class = np.unravel_index(scores[:, 1:].argmax(),
                                                     scores[:, 1:].shape)
        best_class += 1  # compensate for background
        best_score = scores[best_proposal, best_class]
        best_box = boxes[best_proposal, 4:8]
        best_object = self._classes[best_class]

        self._logger.debug('Best score for {} is {:.3f} {} {:.3f}'.format(
            best_object,
            best_score,
            '>=' if best_score > threshold else '<',
            threshold)
        )
        if best_score > threshold:
            return {'id': best_object, 'score': best_score, 'box': best_box}
        return {'id': best_object, 'score': best_score, 'box': None}


if __name__ == '__main__':
    from visualization_utils import draw_detection
    path = '/home/mludersdorfer/software/ws_baxter_pnp/src/baxter_pick_and_place'
    print 'started'
    classes = ('__background__',
               'person', 'bench', 'bottle', 'cup', 'apple', 'orange',
               'remote', 'book')
    od = ObjectDetection(root_dir=path, object_ids=classes)
    od.init_model(warmup=True)

    for img_file in [os.path.join(path, 'data', '%s.jpg' % i)
                     for i in ['004545', '000456', '000542', '001150', '001763']]:
        img = cv2.imread(img_file)
        if img is not None:
            det = od.detect_best(img, 0.8)
            # det = od.detect_object(img, 'person', 0.8)
            if det['box'] is not None:
                draw_detection(img, det)
                cv2.imshow('image', img)
                cv2.waitKey(0)
    cv2.destroyAllWindows()
    print 'ended'

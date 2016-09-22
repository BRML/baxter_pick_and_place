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

import init_paths
import caffe
from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect


# Use RPN for proposals
cfg.TEST.HAS_RPN = True
NMS_THRESH = 0.3

red = (0, 0, 255)  # BGR
black = (0, 0, 0)
white = (255, 255, 255)


def textbox(img, text, org, font_face, font_scale, thickness, color, color_box):
    """Draw a filled box with text placed on top of it.

    :param img: The image to draw on.
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
    cv2.rectangle(img, pt1=(ox - 2, oy + 2), pt2=(ox + 2 + w, oy - 2 - h),
                  color=color_box, thickness=cv2.cv.CV_FILLED)
    cv2.putText(img, text=text, org=(ox, oy),
                fontFace=font_face, fontScale=font_scale,
                thickness=thickness, color=color)


class ObjectDetector(object):
    def __init__(self, root_dir, classes):
        # root_dir: where the baxter_pick_and_place ROS package resides
        # classes: list of objects in set of objects
        self._classes = classes
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

        # Tell Caffe to use the GPU
        caffe.set_mode_gpu()
        gpu_id = 0
        caffe.set_device(gpu_id)
        cfg.GPU_ID = gpu_id
        # Load the network
        self._net = caffe.Net(self._prototxt, self._caffemodel, caffe.TEST)
        logging.info('Loaded network %s.' % self._caffemodel)
        if warmup:
            img = 128 * np.ones((300, 500, 3), dtype=np.uint8)
            for i in xrange(2):
                _, _ = im_detect(self._net, img)

    def detect(self, img):
        # img: numpy array of shape (h, w, 3)
        start = time.time()
        scores, boxes = im_detect(self._net, img)
        logging.info('Detection took {:.3f}s for {:d} object proposals'.format(
            time.time() - start, boxes.shape[0])
        )
        return scores, boxes

    def detect_object(self, img, object_id, threshold=0.5):
        # img: numpy array of shape (h, w, 3)
        # object_id: object identifier string contained in list of objects
        # threshold: threshold on the score
        if object_id not in self._classes:
            raise KeyError("Object {} is not contained in the defined "
                           "set of objects!".format(object_id))
        scores, boxes = self.detect(img=img)
        # Find scores for requested object class
        cls_idx = self._classes.index(object_id)
        cls_scores = scores[:, cls_idx]
        cls_boxes = boxes[:, 4*cls_idx:4*(cls_idx + 1)]
        best_idx = np.argmax(cls_scores)
        best_score = cls_scores[best_idx]
        best_box = cls_boxes[best_idx]
        logging.info('Best score for {} is {:.3f} {} {:.3f}'.format(
            object_id,
            best_score,
            '>=' if best_score > threshold else '<',
            threshold)
        )
        if best_score > threshold:
            return best_score, best_box
        return best_score, None

    @staticmethod
    def draw_detection(img, object_id, scores, boxes):
        if boxes is None:
            return None
        if len(boxes.shape) == 1:
            scores = scores[np.newaxis]
            boxes = boxes[np.newaxis, :]
        for score, box in zip(scores, boxes):
            cv2.rectangle(img, pt1=(box[0], box[1]), pt2=(box[2], box[3]),
                          color=red, thickness=2)
            textbox(img, text='%s %.3f' % (object_id, score),
                    org=(box[0] + 3, box[3] - 3),
                    font_face=cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale=0.5, thickness=2, color=black, color_box=white)


if __name__ == '__main__':
    path = '/home/mludersdorfer/software/ws_baxter_pnp/src/baxter_pick_and_place'
    logging.basicConfig(filename=os.path.join(path, 'afirsttest.log'), level=logging.INFO)
    logging.info('started')
    classes = ('__background__',
               'aeroplane', 'bicycle', 'bird', 'boat',
               'bottle', 'bus', 'car', 'cat', 'chair',
               'cow', 'diningtable', 'dog', 'horse',
               'motorbike', 'person', 'pottedplant',
               'sheep', 'sofa', 'train', 'tvmonitor')
    od = ObjectDetector(root_dir=path, classes=classes)
    od.init_model(warmup=False)

    for img_file in [os.path.join(path, 'data', '%s.jpg' % i)
                     for i in ['004545', '000456', '000542', '001150', '001763']]:
        img = cv2.imread(img_file)
        if img is not None:
            score, box = od.detect_object(img, 'dog', 0.8)
            if box is not None:
                od.draw_detection(img, 'dog', score, box)
                cv2.imshow('image', img)
                cv2.waitKey(0)
    cv2.destroyAllWindows()
    logging.info('ended')


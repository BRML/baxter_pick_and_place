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

import numpy as np

import rospy
from sensor_msgs.msg import Image

import cv2

from settings.debug import (
    topic_img1,
    topic_img2,
    topic_img3,
    topic_img4,
    topic_visualization
)
from hardware import img_to_imgmsg, imgmsg_to_img


def callback(msg):
    try:
        return imgmsg_to_img(imgmsg=msg)
    except ValueError:
        return None


class Inspector(object):
    def __init__(self):
        self._lw = 10
        self._hd = (1080 + self._lw, 1920 + self._lw, 3)
        self._qhd = (1080//2, 1920//2, 3)
        self.rate = rospy.Rate(24)  # visualize images @ 24 Hz

        self.img = np.zeros(self._hd, dtype=np.uint8)
        self.img1 = None
        self.img2 = None
        self.img3 = None
        self.img4 = None

        self._sub1 = rospy.Subscriber(topic_img1, Image, callback=self._cb1, queue_size=1)
        self._sub2 = rospy.Subscriber(topic_img2, Image, callback=self._cb2, queue_size=1)
        self._sub3 = rospy.Subscriber(topic_img3, Image, callback=self._cb3, queue_size=1)
        self._sub4 = rospy.Subscriber(topic_img4, Image, callback=self._cb4, queue_size=1)

        self.pub = rospy.Publisher(topic_visualization, Image, queue_size=1)

    def _cb1(self, msg):
        self.img1 = callback(msg=msg)

    def _cb2(self, msg):
        self.img2 = callback(msg=msg)

    def _cb3(self, msg):
        self.img3 = callback(msg=msg)

    def _cb4(self, msg):
        self.img4 = callback(msg=msg)

    def assemble_image(self):
        """Populate the four quadrants of self.img with images read from the
        four topics defined in settings.debug.topic_img*, where * can be 1,
        2, 3 and 4.

        If any of the images is not available, the previous image is retained.
        """
        if self.img1 is not None:
            self.img[:self._qhd[0], :self._qhd[1], :] = \
                cv2.resize(src=self.img1, dsize=self._qhd[:-1][::-1])
        if self.img2 is not None:
            self.img[:self._qhd[0], self._qhd[1] + self._lw:, :] = \
                cv2.resize(src=self.img2, dsize=self._qhd[:-1][::-1])
        if self.img3 is not None:
            self.img[self._qhd[0] + self._lw:, :self._qhd[1], :] = \
                cv2.resize(src=self.img3, dsize=self._qhd[:-1][::-1])
        if self.img4 is not None:
            self.img[self._qhd[0] + self._lw:, self._qhd[1] + self._lw:, :] = \
                cv2.resize(src=self.img4, dsize=self._qhd[:-1][::-1])

    def publish(self):
        """Publish self.img to the ROS topic defined in
        settings.debug.topic_visualization.
        """
        if topic_visualization == '/robot/xdisplay':
            xsize = (1024, 600)
            if any([(a > b) for a, b in zip(self.img.shape[:2], xsize)]):
                self.img = cv2.resize(src=self.img, dsize=xsize)
        self.pub.publish(img_to_imgmsg(img=self.img))


def main():
    """A ROS node that combines 4 individual images in one big image for
    visual inspection of the demonstration.
    """
    print 'Initializing node ...'
    rospy.init_node('demo_inspection_module')

    print 'Publishing inspection images to {}'.format(topic_visualization)

    insp = Inspector()

    while not rospy.is_shutdown():
        insp.assemble_image()
        insp.publish()
        insp.rate.sleep()


if __name__ == '__main__':
    main()

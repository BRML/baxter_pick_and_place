#!/usr/bin/env python

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

import argparse
import cv2
import os
import rospkg
import rospy
import sys
import tty
import termios

from sensor_msgs.msg import Image

from baxter_pick_and_place.image import _write_img, _imgmsg2img
from baxter_pick_and_place.rand import rand_x_digit_num
from baxter_pick_and_place.robot import Robot
from baxter_pick_and_place.settings import parameters as table


class Images(object):
    def __init__(self, limb, outpath):
        """ Image recorder.
        :param limb: the limb to record images with
        :param outpath: the path to write the images to
        """
        self._arm = limb
        self._outpath = outpath

        self.robot = Robot(self._arm, self._outpath)
        self.robot._camera.exposure = 100
        self._image = None
        self._cam_sub = None

    def record(self):
        """ Records an image each time key 'r' is pressed. Stops upon pressing
        key 's'.
        """
        s = '/cameras/' + self._arm + '_hand_camera/image'
        self._cam_sub = rospy.Subscriber(s, Image, callback=self._camera_callback)
        print "\nRecording images ..."
        while not rospy.is_shutdown():
            # pose = self.robot._perturbe_pose(self.robot._top_pose)
            # self.robot._move_to_pose(pose)
            self.robot._move_to_pose(self.robot._top_pose)
            print " Press 'r' to record image, 's' to stop."
            ch = self.getch()
            if ch == 'r':
                fname = os.path.join(self._outpath, rand_x_digit_num(12))
                _write_img(self._image[0], fname)
                _write_img(self._image[1], fname + '_fil')
                _write_img(self._image[2], fname + '_ahe')
                print " Recorded image '%s.jpg'." % fname
            elif ch == 's':
                self._cam_sub.unregister()
                break

    @staticmethod
    def getch():
        """ getch()-like unbuffered character reading from stdin.
        See http://code.activestate.com/recipes/134892/.
        :return: a single character
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def _camera_callback(self, data):
        """
        Callback routine for the camera subscriber.
        """
        self._image = self._segment_filter(_imgmsg2img(data))

    def _segment_filter(self, image):
        """ Segment and filter / enhance the recorded image.
        :param image: the image to modify
        :return: the modified image
        """
        # hard-coded segmentation of relevant workspace
        seg = image[table['y_min']:table['y_max'],
                    table['x_min']:table['x_max'], :]

        # noise reduction
        fil = cv2.bilateralFilter(seg, d=3, sigmaColor=5, sigmaSpace=5)
        diff = seg - fil

        # adaptive histogram equalization
        ycrcb = cv2.cvtColor(fil, cv2.COLOR_BGR2YCR_CB)
        channels = cv2.split(ycrcb)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        channels[0] = clahe.apply(channels[0])
        ycrcb = cv2.merge(channels)
        cl = cv2.cvtColor(ycrcb, cv2.COLOR_YCR_CB2BGR)

        # noise reduction
        # fil2 = cv2.bilateralFilter(cl, d=5, sigmaColor=5, sigmaSpace=9)
        # diff2 = cl - fil2

        # visualization
        cv2.imshow('%s workspace' % self._arm, seg)
        cv2.imshow('%s denoised' % self._arm, fil)
        cv2.imshow('%s difference' % self._arm, diff)
        cv2.imshow('%s clahe' % self._arm, cl)
        # cv2.imshow('%s clahe difference' % self._arm, diff2)
        # cv2.imshow('%s clahe filtered' % self._arm, fil2)
        cv2.waitKey(3)

        return seg, fil, cl


def main():
    """Image recorder software

    Records an image each time the key 'r' (for 'record') is pressed.
    Upon pressing the key 's' (for 'stop'), the program exits.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='limb to record images with'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospack = rospkg.RosPack()
    ns = rospack.get_path('baxter_pick_and_place')
    ns = os.path.join(ns, 'data', 'sdd')
    if not os.path.exists(ns):
        os.makedirs(ns)

    print("Initializing node... ")
    rospy.init_node("image_recording_%s" % (args.limb,))

    images = Images(args.limb, outpath=ns)
    rospy.on_shutdown(images.robot.clean_shutdown)
    images.record()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

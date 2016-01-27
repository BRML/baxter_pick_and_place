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

# The code in this file is adapted from the 'joint_position_waypoints'-example
# provided by rethink robotics in the Baxter SDK found in the baxter_examples
# repository.

import argparse
import os
import random
import rospkg
import rospy
import sys
import tty
import termios

from baxter_pick_and_place.image import write_imgmsg
from baxter_pick_and_place.robot import Robot


class Images(object):
    def __init__(self, limb, outpath):
        self._arm = limb
        self._outpath = outpath
        self.robot = Robot(self._arm, self._outpath)

    def record(self):
        print "\nRecording images. Press 'Ctrl+C' to stop."
        while not rospy.is_shutdown():
            # TODO: display current image
            # pose = self.robot._perturbe_pose(self.robot._top_pose)
            # self.robot._move_to_pose(pose)
            self.robot._move_to_pose(self.robot._top_pose)
            print "Press 'r' to record image."
            # TODO: getch() is blocking, replace with callback of some sort
            while not rospy.is_shutdown() and not self.getch() == 'r':
                pass
            imgmsg = self.robot._record_image()
            fname = os.path.join(self._outpath, rand_x_digit_num(12)) + '.jpg'
            write_imgmsg(imgmsg, fname)
            print " Recorded image '%s'." % fname

    @staticmethod
    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


def rand_x_digit_num(x, leading_zeroes=True):
    """Return an X digit number, leading_zeroes returns a string, otherwise int
    See http://stackoverflow.com/questions/13496087/
    python-how-to-generate-a-12-digit-random-number"""
    if not leading_zeroes:
        # wrap with str() for uniform results
        return random.randint(10**(x-1), 10**x-1)
    else:
        if x > 6000:
            return ''.join([str(random.randint(0, 9)) for i in xrange(x)])
        else:
            return '{0:0{x}d}'.format(random.randint(0, 10**x-1), x=x)


def main():
    """Image recorder software

    Records an image each time the navigator 'OK/wheel' button is pressed.
    Upon pressing the navigator 'Rethink' button, the program exits.
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
    ns = os.path.join(ns, 'data')
    if not os.path.exists(ns):
        os.makedirs(ns)

    print("Initializing node... ")
    rospy.init_node("image_recording_%s" % (args.limb,))

    images = Images(args.limb, outpath=ns)
    rospy.on_shutdown(images.robot.clean_shutdown)

    # Begin example program
    images.record()

if __name__ == '__main__':
    main()

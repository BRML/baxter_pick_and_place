#!/usr/bin/env python

# Copyright (c) 2015, 2016, BRML
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

import rospy

from baxter_pick_and_place.robot import Robot


class Demonstrator(object):

    def __init__(self, limb):
        """
        Picks up objects pointed out and places them in a pre-defined location.
        """
        self.robot = Robot(limb)
        self._N_TRIES = 2

    def demonstrate(self, n_objects_to_pick):
        """ Pick up a given number of objects and place them in a pre-defined
        location.
        :param n_objects_to_pick: The number of objects to pick up.
        :return: True on completion.
        """
        n = 0
        print '\nWe are supposed to pick up %i object(s) ...' % n_objects_to_pick
        while not rospy.is_shutdown() and n < n_objects_to_pick:
            print "Picking up object", n
            # try up to 3 times to grasp an object
            if self.robot.pick_and_place_object():
                n += 1
            else:
                n_tries = self._N_TRIES
                while n_tries > 0:
                    print ' trying', n_tries, 'more time(s)'
                    n_tries -= 1
                    if self.robot.pick_and_place_object():
                        n_tries = -1
                if not n_tries == -1:
                    print 'Failed to pick up object', n
                    return False
        return True


def main():
    """ Pick and place demonstration with the baxter research robot.

    Picks up objects that have been pointed out by a human operator by means
    of an eye tracker and places them in a pre-defined location.
    """
    parser = argparse.ArgumentParser(
        description='Pick and place demonstration with the baxter research robot.')
    parser.add_argument('-l', '--limb', dest='limb', required=False,
                        choices=['left', 'right'], default='left',
                        help='The limb to pick objects up with.')
    parser.add_argument('-n', '--number', dest='number',
                        required=False, type=int, default=0,
                        help='The number of objects to pick up.')
    args = parser.parse_args(rospy.myargv()[1:])

    print 'Initializing node ...'
    rospy.init_node('baxter_pick_and_place_demonstrator')

    demonstrator = Demonstrator(args.limb)
    rospy.on_shutdown(demonstrator.robot.clean_shutdown)
    ret = demonstrator.demonstrate(args.number)
    if ret:
        print "\nSuccessfully performed demonstration."
    else:
        print "\nFailed demonstration."

    print "\nDone with experiment. Press 'Ctrl-C' to exit."
    rospy.spin()

if __name__ == '__main__':
    main()

#!/usr/bin/env python

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

import argparse

import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION


class Demonstrator(object):

    def __init__(self):
        """
        Picks up objects pointed out and places them in a pre-defined location.
        """
        print("Getting robot state ... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def clean_shutdown(self):
        print("\nExiting demonstrator ...")
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def demonstrate(self, n_objects_to_pick):
        """ Pick up a given number of objects and place them in a pre-defined
        location.

        :param n_objects_to_pick: The number of objects to pick up.
        :return: True on completion.
        """
        n = 0
        print 'We are supposed to pick up %i objects ...' % n_objects_to_pick
        while not rospy.is_shutdown() and n < n_objects_to_pick:
            print "Picking up object", n
            n += 1
        return True


def main():
    """ Pick and place demonstration with the baxter research robot.

    Picks up objects that have been pointed out by a human operator by means
    of an eye tracker and places them in a pre-defined location.
    """
    parser = argparse.ArgumentParser(description='Pick and place demonstration with the baxter research robot.')
    parser.add_argument('-n', '--number', dest='number',
                        required=False, type=int, default=0,
                        help='The number of objects to pick up.')
    args = parser.parse_args(rospy.myargv()[1:])

    print 'Initializing node ...'
    rospy.init_node('baxter_pick_and_place_demonstrator')

    demonstrator = Demonstrator()
    rospy.on_shutdown(demonstrator.clean_shutdown)
    demonstrator.demonstrate(args.number)

    print 'Done.'

if __name__ == '__main__':
    main()

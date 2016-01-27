#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import cv2
import cv_bridge
import random

import rospy
from sensor_msgs.msg import Image

import baxter_interface


class Waypoints(object):
    def __init__(self, limb):
        # Create baxter_interface limb instance
        self._arm = limb
        self._limb = baxter_interface.Limb(self._arm)
        self._camera = baxter_interface.CameraController('%s_hand_camera' % self._arm)
        self._camera.resolution = (960, 600)
        # Recorded waypoints
        self._imgmsg = None
        self._image = None

        # Recording state
        self._is_recording = False

        # Verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # Create Navigator I/O
        self._navigator_io = baxter_interface.Navigator(self._arm)

    def _record_image(self, value):
        """
        Record an image from one of the robots hand cameras.
        """
        if value:
            try:
                self._image = cv_bridge.CvBridge().imgmsg_to_cv2(self._imgmsg, 'rgb8')
            except cv_bridge.CvBridgeError:
                raise
            fname = rand_x_digit_num(12)
            cv2.imwrite(fname + '.jpg', self._image)
            print "recorded image '%s.jpg'" % fname

    def _stop_recording(self, value):
        """
        Sets is_recording to false

        Navigator 'Rethink' button callback
        """
        # On navigator Rethink button press, stop recording
        if value:
            self._is_recording = False

    def _camera_callback(self, data):
        """
        Callback routine for the camera subscriber.
        """
        self._imgmsg = data

    def record(self):
        """
        Records joint position waypoints upon each Navigator 'OK/Wheel' button
        press.
        """
        rospy.loginfo("Image Recording Started")
        print("Press Navigator 'OK/Wheel' button to record a new image.")
        print("Press Navigator 'Rethink' button when finished recording.")
        # Connect Navigator I/O signals
        # Navigator scroll wheel button press
        self._navigator_io.button0_changed.connect(self._record_image)
        # Navigator Rethink button press
        self._navigator_io.button2_changed.connect(self._stop_recording)

        # Set recording flag
        self._is_recording = True

        # Loop until waypoints are done being recorded ('Rethink' Button Press)
        s = '/cameras/' + self._arm + '_hand_camera/image'
        cam_sub = rospy.Subscriber(s, Image, callback=self._camera_callback)
        while self._is_recording and not rospy.is_shutdown():
            rospy.sleep(1.0)

        cam_sub.unregister()
        # We are now done with the navigator I/O signals, disconnecting them
        self._navigator_io.button0_changed.disconnect(self._record_image)
        self._navigator_io.button2_changed.disconnect(self._stop_recording)

    def clean_shutdown(self):
        print("\nExiting image recording...")
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True


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
        help='limb to record/playback waypoints'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("image_recording_%s" % (args.limb,))

    waypoints = Waypoints(args.limb)

    # Register clean shutdown
    rospy.on_shutdown(waypoints.clean_shutdown)

    # Begin example program
    waypoints.record()

if __name__ == '__main__':
    main()

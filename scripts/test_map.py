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

from baxter_pick_and_place.image import imgmsg2img
from baxter_pick_and_place.robot import Robot
from baxter_pick_and_place.settings import parameters as table


def put_label(img, text, org, font_face, font_scale, fgc, bgc):
    size = cv2.getTextSize(text=text,
                           fontFace=font_face, fontScale=font_scale,
                           thickness=1)
    w, h = size[0]
    o = 3
    ll = (org[0]-o, org[1]+o)
    ur = (org[0]+w+o, org[1]-h-o)
    cv2.rectangle(img, pt1=ll, pt2=ur, color=bgc, thickness=cv2.cv.CV_FILLED)
    cv2.putText(img, text=text, org=org,
                fontFace=font_face, fontScale=font_scale, color=fgc)


def click(event, x, y, flags, params):
    color = (0, 255, 0)
    if event == cv2.EVENT_LBUTTONDOWN:
        cv2.circle(image, (x, y), 3, color, 2)
        nx = table['x_min']+x
        ny = table['y_min']+y
        px, py, pz, ox, oy, oz = robot._pixel2position((nx, ny))
        text = "x: %.2f, y: %.2f, z:%.2f" % (px, py, pz)
        put_label(image, text=text, org=(5, image.shape[0]-5),
                  font_face=cv2.FONT_HERSHEY_SIMPLEX, font_scale=1.0,
                  fgc=color, bgc=(0, 0, 0))
        cv2.imshow("image", image)


doc = "Test pixel-to-world mapping with a hand camera of the baxter " + \
      "research robot."
arg_fmt = argparse.RawDescriptionHelpFormatter
parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                 description=doc)
required = parser.add_argument_group('required arguments')
required.add_argument(
    '-l', '--limb', required=True, choices=['left', 'right'],
    help='The limb to pick objects up with.'
)
args = parser.parse_args(rospy.myargv()[1:])

ns = rospkg.RosPack().get_path('baxter_pick_and_place')
data_dirname = os.path.join(ns, 'data')
if not os.path.exists(data_dirname):
    os.makedirs(data_dirname)

print 'Initializing node ...'
rospy.init_node('baxter_pick_and_place_map')

robot = Robot(args.limb, outpath=data_dirname)
rospy.on_shutdown(robot.clean_shutdown)

robot._perform_setup()
robot.move_to_pose(robot._top_pose)
image = imgmsg2img(robot._record_image())
cv2.namedWindow("image")
cv2.setMouseCallback("image", click)

print "\nClick in the 'image'-window to compute real world coordinates."
print "Press 'r' to refresh the image."
print "Press 'c' to stop."

while not rospy.is_shutdown():
    cv2.imshow("image", image)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("r"):
        image = imgmsg2img(robot._record_image())
    elif key == ord("c"):
        cv2.destroyWindow("image")
        break

print "\nDone. Press 'Ctrl-C' to exit."
rospy.spin()
cv2.destroyAllWindows()

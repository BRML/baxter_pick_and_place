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


import rospkg
import rospy

from demo import PickAndPlace, settings
from hardware import Baxter, Kinect
from simulation import sim_or_real, Environment
from vision import ObjectDetection, ObjectSegmentation


class Demonstration(object):
    def __init__(self, ros_ws, object_set):
        """Demonstration class setting up all the components of the
        demonstration framework and performing the demonstration task.

        :param ros_ws: The path to the baxter_pick_and_place ROS package.
        :param object_set: The list of object identifiers comprising the set
            of objects, prepended by a background class.
        """
        self._sim = sim_or_real()
        self._environment = Environment(root_dir=ros_ws,
                                        # TODO replace with object_set[1:],
                                        object_ids=['duplo_brick', 'robot'],
                                        ws_limits=settings.workspace_limits_m)
        self._robot = Baxter(sim=self._sim)
        self._camera = Kinect()
        self._detection = ObjectDetection(root_dir=ros_ws,
                                          object_ids=object_set)
        self._segmentation = ObjectSegmentation(root_dir=ros_ws,
                                                object_ids=object_set)
        self._demo = PickAndPlace(robot=self._robot,
                                  camera=self._camera,
                                  detection=self._detection,
                                  segmentation=self._segmentation)
        #  register visual servoing module (requires robot, detection, segmentation instances, ...)

    def shutdown_routine(self):
        """Clean up everything that needs cleaning up before ROS is shutdown."""
        self._robot.clean_up()
        if self._sim:
            self._environment.clean_up()

    def set_up(self):
        """Prepare all the components of the demonstration."""
        self._robot.set_up()
        if self._sim:
            self._environment.set_up()
        # self._detection.init_model(warmup=True)
        # self._segmentation.init_model(warmup=True)
        self._demo.calibrate()

    def demonstrate(self):
        """Perform the demonstration."""
        self._demo.perform()


if __name__ == '__main__':
    print 'Initializing node ...'
    rospy.init_node('demo_module')
    ns = rospkg.RosPack().get_path('baxter_pick_and_place')

    demo = Demonstration(ros_ws=ns, object_set=settings.object_ids)
    rospy.on_shutdown(demo.shutdown_routine)
    demo.set_up()
    demo.demonstrate()


# def main():
#     from visual.image import imgmsg2img
#     from vision.detection import ObjectDetector
#     import cv2
#     import logging
#
#     # forward faster RCNN object detection logger to ROS
#     import rosgraph.roslogging as _rl
#     logging.getLogger('frcnn').addHandler(_rl.RosStreamHandler())
#     import rospy.impl.rosout as _ro
#     logging.getLogger('frcnn').addHandler(_ro.RosOutHandler())

#     od = ObjectDetector(root_dir=ns, classes=settings.object_ids)
#     od.init_model(warmup=False)
#     while not rospy.is_shutdown():
#         imgmsg = demonstrator.robot._record_image()
#         img = imgmsg2img(imgmsg)
#         if img is not None:
#             print img.shape
#             cv2.imshow('raw', img)
#             score, box = od.detect_object(img, 'person', 0.8)
#             if box is not None:
#                 od.draw_detection(img, 'person', score, box)
#                 cv2.imshow('image', img)
#         cv2.waitKey(1)
#     cv2.destroyAllWindows()
#
#
# if __name__ == '__main__':
#     main()

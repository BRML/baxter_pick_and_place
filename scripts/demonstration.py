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

import datetime
import logging
import os

import rospkg
import rospy
from sensor_msgs.msg import Image

from core import get_default_handler
from demo import PickAndPlace, settings
from hardware import Baxter, Kinect
from servoing import ServoingDistance, ServoingSize
from simulation import sim_or_real, Environment
from vision import ObjectSegmentation


class Demonstration(object):
    def __init__(self, ros_ws, object_set):
        """Demonstration class setting up all the components of the
        demonstration framework and performing the demonstration task.

        :param ros_ws: The path to the baxter_pick_and_place ROS package.
        :param object_set: The list of object identifiers comprising the set
            of objects, prepended by a background class.
        """
        self._logger = logging.getLogger('main')

        self._sim = sim_or_real()
        self._environment = Environment(root_dir=ros_ws,
                                        # TODO replace with object_set[1:],
                                        object_ids=['duplo_brick', 'robot'],
                                        ws_limits=settings.task_space_limits_m)
        self._robot = Baxter(sim=self._sim)
        self._camera = Kinect(root_dir=ros_ws, host=settings.elte_kinect_win_host)
        self._detection = None
        self._segmentation = ObjectSegmentation(root_dir=ros_ws,
                                                object_ids=object_set)

        self._logger.info("Will publish visualization images to topic "
                          "'{}'.".format(settings.topic_visualization))
        pub_vis = rospy.Publisher(settings.topic_visualization, Image,
                                  queue_size=10, latch=True)
        self._servo = {
            'table': ServoingDistance(robot=self._robot,
                                      segmentation=self._segmentation,
                                      pub_vis=pub_vis,
                                      object_size=settings.object_size_meters,
                                      tolerance=settings.servo_tolerance_meters),
            'hand': ServoingSize(robot=self._robot,
                                 segmentation=self._segmentation,
                                 pub_vis=pub_vis,
                                 object_size=settings.object_size_meters,
                                 tolerance=settings.servo_tolerance_meters)
        }
        self._demo = PickAndPlace(robot=self._robot,
                                  servo=self._servo,
                                  camera=self._camera,
                                  detection=self._detection,
                                  segmentation=self._segmentation,
                                  pub_vis=pub_vis,
                                  root_dir=ros_ws)

    def shutdown_routine(self):
        """Clean up everything that needs cleaning up before ROS is shutdown."""
        self._logger.info('Shut down the demonstration framework.')
        self._robot.clean_up()
        if self._sim:
            self._environment.clean_up()

    def set_up(self):
        """Prepare all the components of the demonstration."""
        self._logger.info('Set up the demonstration framework.')
        self._robot.set_up()
        if self._sim:
            self._environment.set_up()
        self._segmentation.init_model(warmup=True)
        self._demo.calibrate()

    def demonstrate(self):
        """Perform the demonstration."""
        self._logger.info('Perform the demonstration.')
        self._demo.perform()
        self._logger.info('Done.')


if __name__ == '__main__':
    ns = rospkg.RosPack().get_path('baxter_pick_and_place')

    logfolder = os.path.join(ns, 'log')
    if not os.path.exists(logfolder):
        os.makedirs(logfolder)
    filename = datetime.datetime.now().strftime(format="%Y%m%d_%H%M")
    logfile = os.path.join(logfolder, '{}_demo.log'.format(filename))
    logfile = ''

    logger = logging.getLogger('main')
    logger.setLevel(logging.DEBUG)
    hdlr = get_default_handler(filename=logfile, level=logging.DEBUG)
    for h in hdlr:
        logger.addHandler(hdlr=h)

    print 'Initialize ROS node.'
    rospy.init_node('demo_module')
    logger.info('Initialize demonstration framework.')
    demo = Demonstration(ros_ws=ns, object_set=settings.object_ids)
    rospy.on_shutdown(demo.shutdown_routine)
    demo.set_up()
    demo.demonstrate()

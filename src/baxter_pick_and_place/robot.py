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

import numpy as np
import os
import pickle
import time

import rospy

from sensor_msgs.msg import Image

import baxter_interface

from baxter_pick_and_place.image import (
    cut_imgmsg,
    white_imgmsg,
    write_imgmsg,
    segment_area
)
from baxter_pick_and_place.baxter_robot import BaxterRobot
from baxter_pick_and_place.settings import parameters as table
from baxter_pick_and_place.settings import top_pose


class Robot(BaxterRobot):

    def __init__(self, limb, outpath):
        """
         A baxter research robot instance with some additional functionality.
         :param limb: The limb to use for the demonstration.
         :param outpath: The path to write output files into.
        """
        BaxterRobot.__init__(self, limb=limb)
        self._outpath = outpath

        self._imgmsg = None
        self._cam_pars = None

        self._top_pose = top_pose
        self._bin_pose = None
        self._N_TRIES = 2

        self.display_image(white_imgmsg())
        self._limb.set_joint_position_speed(0.5)

    def clean_shutdown(self):
        """ Clean shutdown of the robot.
        :return: True on completion
        """
        print "\nExiting demonstrator ..."
        self.display_image(white_imgmsg())
        self.move_to_pose(self._top_pose)
        self._limb.move_to_neutral()
        if not self._init_state:
            print "Disabling robot..."
            self._rs.disable()
        return True

    """ =======================================================================
        Set system up and prepare things
    ======================================================================= """
    def set_up(self):
        self._perform_setup(finger='short')
        print self._cam_pars['dist'], self._cam_pars['z_offset']
        self._detect_bin()
        self._approach_pose(self._bin_pose)
        self.move_to_pose(self._bin_pose)

    def _perform_setup(self, finger='short'):
        """ Perform the robot limb calibration, i.e., measure the distance from
        the distance sensor to the table and set some other parameters, if no
        setup file exists. If a setup file exists, load it.

        Note: To force performing a new camera setup delete the old setup file.
        :param finger: Fingers used for baxter gripper ['long', 'short'].

        Note: Obviously this cannot replace a full camera calibration and
        proper pose estimation, but for the purpose of this demonstration the
        implemented solution adapted from
          http://sdk.rethinkrobotics.com/wiki/Worked_Example_Visual_Servoing
        is sufficient.
        """
        spath = os.path.join(self._outpath, 'setup')
        if not os.path.exists(spath):
            os.makedirs(spath)
        sfile = os.path.join(spath, 'setup.pkl')

        if not os.path.exists(sfile):
            pose = [0.60, 0.20, 0.0, -1.0*np.pi, 0.0*np.pi, 0.0*np.pi]
            n_calibrations = 10
            self._cam_pars = dict()

            dist = list()
            sensor = baxter_interface.analog_io.AnalogIO(self._arm +
                                                         '_hand_range')
            print '\nPerforming camera setup ...'
            while len(dist) < n_calibrations:
                p = self._perturbe_pose(pose)
                if self.move_to_pose(p):
                    d = sensor.state()
                    if d < 65000:
                        dist.append(d/1000.0 - p[2])
                        print ' recorded %i/%i measurements (d=%.3fm)' % \
                              (len(dist), n_calibrations, dist[-1])
                    else:
                        print ' ERROR: no valid distance found'
            self._cam_pars['dist'] = np.mean(dist)
            print 'Will be working with average distance d=%.3fm.' % \
                  self._cam_pars['dist']

            self._cam_pars['mpp'] = 0.0025  # meters per pixel @ 1m
            self._cam_pars['x_offset'] = 0.01
            self._cam_pars['y_offset'] = -0.02
            bfh_short = 0.073 + 0.003
            bfh_long = 0.112 + 0.003
            bfh = bfh_short if finger is 'short' else bfh_long
            self._cam_pars['z_offset'] = (
                0.025 +  # distance camera--palm
                0.01 +  # safety offset
                bfh +  # height of fingers
                0.015  # magic constant
            )

            with open(sfile, 'w') as f:
                pickle.dump(self._cam_pars, f)
        else:
            print '\nLoading camera setup ...'
            with open(sfile, 'r') as f:
                self._cam_pars = pickle.load(f)

    def _detect_bin(self):
        """ Detect the bin to put the objects into. """
        print '\nLooking for bin to put objects into ...'
        self.move_to_pose(self._top_pose)
        # record top-down-view image
        imgmsg = self._record_image()
        write_imgmsg(imgmsg, os.path.join(self._outpath, 'bin_top_view'))
        rroi, roi = segment_area(imgmsg=imgmsg, outpath=self._outpath,
                                 th=210, c_low=110, c_high=175,
                                 a_low=10000, a_high=100000)
        x, y, w, h = roi
        center = table['x_min']+x+w/2, table['y_min']+y+h/2
        # center = (571, 488)
        print ' Found bin at (%i, %i) pixels.' % (int(center[0]),
                                                  int(center[1]))
        print ' Computing baxter coordinates from pixel coordinates ...'
        self._bin_pose = self._pixel2position(center)
        print 'Detected bin at (%.2f, %.2f, %.2f) m.' % (self._bin_pose[0],
                                                         self._bin_pose[1],
                                                         self._bin_pose[2])

    """ =======================================================================
        Pick and place routine
    ======================================================================= """
    def pick_and_place_object(self):
        """ Detect, pick up and place an object upon receiving an execution
        command.
        :return: boolean flag on completion
        """
        self.set_up()

        print ' rabbiting away ...'
        # Wait for object to be triggered---dummy

        def _trigger_dummy(max_time):
            t = np.random.uniform(0.0, max_time)
            time.sleep(t)
            idx = np.random.randint(0, 5)
            print "  object '%i' was triggered after %.2fs" % (idx, t)
            return idx

        object_id = _trigger_dummy(10.0)

        # move limb to top-down-view pose
        # try up to 3 times to find a valid pose
        if not self.move_to_pose(self._top_pose):
            n_tries = self._N_TRIES
            while n_tries > 0:
                print '  trying', n_tries, 'more time(s)'
                n_tries -= 1
                ppose = self._perturbe_pose(self._top_pose)
                if self.move_to_pose(ppose):
                    print '  perturbation worked'
                    n_tries = -1
            if not n_tries == -1:
                return False
        return self._try_object(object_id=object_id)

    def _try_object(self, object_id):
        """ Try to select, pick up and place the target object.
        :param object_id: the id of the desired object in the data base.
        :return: boolean flag on completion
        """
        # record top-down-view image
        imgmsg = self._record_image()
        self.display_image(imgmsg)
        # candidates = detect_object_candidates(imgmsg)
        # ignore candidates already in bin
        # candidate = candidates[0]
        # center, dots = candidate
        # pose = self._pixel2position(center)
        # r, p, y = self._estimate_grasp_orientation(dots, object_id)
        # pose[3] = r
        # pose[4] = p
        # pose[5] = y
        pose = [0.54, -0.11, -0.26, 1.0*np.pi, 0.0, 0.0]
        if not self._pick_and_place(pose, object_id=object_id):
            n_tries = self._N_TRIES
            while n_tries > 0:
                print '   trying', n_tries, 'more time(s)'
                n_tries -= 1
                ppose = self._perturbe_pose(pose)
                if self._pick_and_place(ppose, object_id=object_id):
                    n_tries = -1
            if not n_tries == -1:
                print "  failed to grasp object '%i'" % object_id
                return False
        return True

    def _pick_and_place(self, pose, object_id):
        """ Try to approach, grasp, relocate and put down an object.
        :param pose: The pose of the selected object.
        :param object_id: The id of the object in the data base.
        :return: Boolean flag on completion.
        """
        self._approach_pose(pose)
        self.move_to_pose(pose)
        if self.grasp_object():
            print '   grasped object'
            self.move_to_pose(self._top_pose)
            bin_pose = self._perturbe_pose(self._bin_pose)
            self._approach_pose(bin_pose)
            self.move_to_pose(bin_pose)
            self.release_object()
            print '   released object'
            self.move_to_pose(self._top_pose)
            print "  object '%i' placed successfully" % object_id
            return True
        print '  missed object'
        self.release_object()
        return False

    """ =======================================================================
        Helper functions and kinematics
    ======================================================================= """
    def _record_image(self):
        """ Record an image from one of the robots' hand cameras.
        :return: a ROS image message
        """
        s = '/cameras/' + self._arm + '_hand_camera/image'
        self._imgmsg = None
        cam_sub = rospy.Subscriber(s, Image, callback=self._camera_callback)
        while self._imgmsg is None:
            time.sleep(0.1)
        cam_sub.unregister()
        return cut_imgmsg(self._imgmsg, **table)

    def _camera_callback(self, data):
        """
        Callback routine for the camera subscriber.
        """
        self._imgmsg = data

    def _pixel2position(self, pixel):
        """ Compute Cartesian position in base coordinates from image pixels.
        Adapted from
          http://sdk.rethinkrobotics.com/wiki/Worked_Example_Visual_Servoing.
        :param pixel: A pixel coordinate
        :return: A pose [x, y, z, 0, 0, 0]
        """
        w, h = self._camera.resolution
        px, py, _, _, _, _ = self._endpoint_pose()
        # x is front/back, so aligned with image height
        x = (pixel[1] - h/2)*self._cam_pars['mpp']*self._cam_pars['dist'] + \
            px + self._cam_pars['x_offset']
        # y is left/right, so aligned with image width
        y = (pixel[0] - w/2)*self._cam_pars['mpp']*self._cam_pars['dist'] + \
            py + self._cam_pars['y_offset']
        z = - self._cam_pars['dist'] + self._cam_pars['z_offset']
        return [x, y, z, -1.0*np.pi, 0., 0.]

    def _approach_pose(self, pose=None):
        """ Move robot limb to Cartesian pose, except for an offset in
        z-direction.
        :type pose: [float, float, float, float, float, float]
        :param pose: desired Cartesian pose to approach
        :return: boolean flag on completion
        """
        # TODO: set approach offset here:
        offset = [0.0, 0.0, 0.05, 0.0, 0.0, 0.0]
        return self.move_to_pose(self._modify_pose(offset=offset, pose=pose))

    def _perturbe_pose(self, pose=None):
        """ Add a small perturbation to the Cartesian pose of the robot.
        :type pose: [float, float, float, float, float, float]
        :param pose: desired Cartesian pose
        :return: perturbed pose
        """
        # TODO: set perturbation amplitude here:
        perturbation = [
            np.random.random()*0.02,
            np.random.random()*0.03,
            np.random.random()*0.02,
            np.random.random()*2.0/180.0,
            np.random.random()*2.0/180.0,
            np.random.random()*2.0/180.0
        ]
        return self._modify_pose(offset=perturbation, pose=pose)

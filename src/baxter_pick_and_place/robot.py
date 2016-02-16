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

from baxter_pick_and_place.baxter_robot import BaxterRobot
from baxter_pick_and_place.image import (
    cut_imgmsg,
    white_imgmsg,
    write_imgmsg,
    segment_area,
    segment_red_area,
    mask_imgmsg_region
)
from baxter_pick_and_place.settings import (
    object_list,
    parameters as table,
    setup_pose,
    top_pose,
    vs_tolerance
)


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
        self._setup_pose = setup_pose

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
            n_calibrations = 10
            self._cam_pars = dict()

            dist = list()
            sensor = baxter_interface.analog_io.AnalogIO(self._arm +
                                                         '_hand_range')
            print '\nPerforming camera setup ...'
            while len(dist) < n_calibrations:
                p = self._perturbe_pose(self._setup_pose)
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
        self.display_image(imgmsg)
        write_imgmsg(imgmsg, os.path.join(self._outpath, 'bin_top_view'))
        rroi, _ = self._segment_bin_roi(imgmsg)
        self._bin_pose = self._rroi2pose(rroi=rroi, obj='bin')

    """ =======================================================================
        Pick and place routine
    ======================================================================= """
    def pick_and_place_object(self):
        """ Detect, pick up and place an object upon receiving an execution
        command.
        :return: boolean flag on completion
        """
        print ' rabbiting away ...'
        # Wait for object to be triggered---dummy

        def _trigger_dummy(max_time):
            t = np.random.uniform(0.0, max_time)
            time.sleep(t)
            idx = np.random.randint(0, 5)
            print "  object '%i' was triggered after %.2fs" % (idx, t)
            return idx

        _trigger_dummy(10.0)
        obj = object_list[0]  # want to test duplo_brick

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
        return self._try_object(obj=obj)

    def _try_object(self, obj):
        """ Try to select, pick up and place the target object.
        :param obj: a string identifying the object.
        :return: boolean flag on completion
        """
        # record top-down-view image
        imgmsg = self._record_image()
        # detect object
        try:
            pose = self._detect_object(imgmsg, obj=obj)
        except ValueError as e:
            rospy.logerr(e)
            return False
        # modify pose to help visual servoing and perform visual servoing
        pose[0] -= 0.07
        pose[5] = 0.0*np.pi
        self._approach_pose(pose)
        rroi = self.visual_servoing()
        pose = self._rroi2pose(rroi, obj=obj)
        # pick up object
        if not self._pick_and_place(pose, obj=obj):
            n_tries = self._N_TRIES
            while n_tries > 0:
                print '   trying', n_tries, 'more time(s)'
                n_tries -= 1
                ppose = self._perturbe_pose(pose)
                if self._pick_and_place(ppose, obj=obj):
                    n_tries = -1
            if not n_tries == -1:
                print "  failed to grasp '%s'." % obj
                return False
        return True

    def _pick_and_place(self, pose, obj):
        """ Try to approach, grasp, relocate and put down an object.
        :param pose: The pose of the selected object.
        :param obj: a string identifying the object.
        :return: Boolean flag on completion.
        """
        self._approach_pose(pose)
        self.move_to_pose(pose)
        if self.grasp_object():
            print '   grasped object'
            self._approach_pose(pose)
            self.move_to_pose(self._top_pose)
            bin_pose = self._perturbe_pose(self._bin_pose)
            self._approach_pose(bin_pose)
            self.release_object()
            time.sleep(0.5)
            print '   released object'
            self.move_to_pose(self._top_pose)
            print "  '%s' placed successfully" % obj
            return True
        print '  missed object'
        self.release_object()
        return False

    """ =======================================================================
        Object detection
    ======================================================================= """
    def _segment_bin_roi(self, imgmsg):
        """ Compute the roteted rectangle encompassing the bin.
        :param imgmsg: a ROS image message
        :return: a rotated region (rroi, corners)
        """
        outpath = os.path.join(self._outpath, 'bin')
        th = 210
        c_low = 110
        c_high = 175
        a_low = 10000
        a_high = 100000
        try:
            rroi, _ = segment_area(imgmsg=imgmsg, outpath=outpath, th=th,
                                   c_low=c_low, c_high=c_high,
                                   ff_connectivity=4,
                                   a_low=a_low, a_high=a_high)
        except ValueError as e:
            rospy.logerr(e)
            raise Exception("No bin found!")
        return rroi

    def _mask_bin_roi(self, imgmsg):
        """ Mask the region of the bin to prevent detecting already picked-up
        objects.
        :param imgmsg: a ROS image message
        :return: a ROS image message
        """
        _, corners = self._segment_bin_roi(imgmsg)
        imgmsg = mask_imgmsg_region(imgmsg, corners=corners)
        self.display_image(imgmsg)
        return imgmsg

    def _detect_object(self, imgmsg, obj):
        """ Detect an object and compute the pose to grasp it.
        :param imgmsg: a ROS image message
        :param obj: a string identifying the object to detect
        :return: the pose [x, y, z, a, b, c] to grasp the object
        """
        imgmsg = self._mask_bin_roi(imgmsg)

        if obj is 'duplo_brick':
            rroi, _ = self._segment_duplo_brick_roi(imgmsg, obj)
        elif obj is 'extra_mints':
            rroi = ((0, 0), (1, 1), 40)
        elif obj is 'glue_stick':
            rroi = ((0, 0), (1, 1), 40)
        elif obj is 'golf_ball':
            rroi = ((0, 0), (1, 1), 40)
        elif obj is 'robot':
            rroi = ((0, 0), (1, 1), 40)
        else:
            s = "I do not know '%s'!" % obj
            rospy.logerr(s)
            raise ValueError(s)
        return self._rroi2pose(rroi=rroi, obj=obj)

    def _segment_duplo_brick_roi(self, imgmsg, obj):
        """ Compute the rotated rectangle encompassing the duplo brick.
        :param imgmsg: a ROS image message
        :param obj: a string identifying the object
        :return a rotated region (rroi, corners)
        """
        outpath = os.path.join(self._outpath, obj)
        th = 25
        c_low = 100
        c_high = 170
        a_low = 50
        a_high = 2100
        rroi, _ = segment_red_area(imgmsg=imgmsg, outpath=outpath, th=th,
                                   c_low=c_low, c_high=c_high,
                                   a_low=a_low, a_high=a_high)
        return rroi

    """ =======================================================================
        Helper functions
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

    def _current_height(self):
        """ Compute current height of wrist over table.
        :return: current height over table in [m]
        """
        z_table = -(self._cam_pars['dist'] - self._setup_pose[2])
        z_curr = self._endpoint_pose()[2]
        return z_curr - z_table

    def _pixel2position(self, pixel):
        """ Compute Cartesian position in base coordinates from image pixels.
        Adapted from
          http://sdk.rethinkrobotics.com/wiki/Worked_Example_Visual_Servoing.
        :param pixel: A pixel coordinate
        :return: A pose [x, y, z, -PI, 0, 0]
        """
        w, h = self._camera.resolution
        px, py, _, _, _, _ = self._endpoint_pose()
        # x is front/back, so aligned with image height
        x = (pixel[1] - h/2)*self._cam_pars['mpp']*self._current_height() + \
            px + self._cam_pars['x_offset']
        # y is left/right, so aligned with image width
        y = (pixel[0] - w/2)*self._cam_pars['mpp']*self._current_height() + \
            py + self._cam_pars['y_offset']
        z_table = -(self._cam_pars['dist'] - self._setup_pose[2])
        z = z_table + self._cam_pars['z_offset']
        return [x, y, z, -1.0*np.pi, 0., 0.]

    def _rroi2pose(self, rroi, obj):
        """ Compute object pose from that object's rotated region.
        :param rroi: rotated region ((cx, cy), (w, h), alpha)
        :param obj: a string identifying the object
        :return the pose [x, y, z, a, b, c] to grasp the object
        """
        center = table['x_min']+rroi[0][0], table['y_min']+rroi[0][1]
        print " Found '%s' at (%i, %i) pixels." % (obj, int(center[0]),
                                                   int(center[1]))
        print ' Computing baxter coordinates from pixel coordinates ...'
        pose = self._pixel2position(center)
        pose[5] -= np.deg2rad(rroi[2])
        print "Detected '%s' at (%.2f, %.2f, %.2f)m and %.2frad." % \
              (obj, pose[0], pose[1], pose[2], pose[5])
        return pose

    def _approach_pose(self, pose=None):
        """ Move robot limb to Cartesian pose, except for an offset in
        z-direction.
        :type pose: [float, float, float, float, float, float]
        :param pose: desired Cartesian pose to approach
        :return: boolean flag on completion
        """
        # TODO: set approach offset here:
        offset = [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
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
            0.0,
            0.0,
            0.0,
            np.random.random()*2.0/180.0
        ]
        return self._modify_pose(offset=perturbation, pose=pose)

    def visual_servoing(self):
        """ Perform visual servoing to position camera over object center.
        return: rotated roi [(cx, cy), (w, h), alpha] of blob
        """
        rroi = self._vs_find_center()
        world_error = 2*vs_tolerance
        while world_error > vs_tolerance:
            rroi, world_error = self._vs_iterate(rroi)
        return rroi

    def _vs_iterate(self, rroi):
        """ Perform one iteration of visual servoing.
        :param rroi: rotated roi [(cx, cy), (w, h), alpha] of blob
        :return: rotated roi [(cx, cy), (w, h), alpha] of blob, deviation
        from image center in world coordinates
        """
        kp = 0.7  # proportional control parameter

        w = table['x_max'] - table['x_min']
        h = table['y_max'] - table['y_min']
        world_error = self._vs_error(pixel_center=rroi[0], size=(w, h))
        if world_error > vs_tolerance:
            pixel_delta = [a-b for a, b in zip((w/2, h/2), rroi[0])]
            factor = self._cam_pars["mpp"]*self._current_height()
            dx = -pixel_delta[1]*factor * kp
            dy = -pixel_delta[0]*factor * kp
            pose = self._modify_pose(offset=[dx, dy, 0, 0, 0, 0])
            self.move_to_pose(pose)
            try:
                rroi = self._vs_find_center()
                world_error = self._vs_error(pixel_center=rroi[0],
                                             size=(w, h))
            except ValueError as e:
                rospy.logerr(e)
        return rroi, world_error

    def _vs_error(self, pixel_center, size):
        """ Compute position error of blob for visual servoing.
        :param pixel_center: blob center in pixel coordinates
        :param size: image size
        :return: error in world coordinates
        """
        w, h = size
        pixel_delta = [a-b for a, b in zip((w/2, h/2), pixel_center)]
        pixel_error = np.sqrt(np.sum(np.asarray(pixel_delta)**2))
        factor = self._cam_pars["mpp"]*self._current_height()
        world_error = pixel_error*factor
        return world_error

    def _vs_find_center(self):
        """ Compute center of blob for visual servoing.
        :return: center of blob in pixel coordinates
        """
        import cv2
        from baxter_pick_and_place.image import imgmsg2img, _img2imgmsg
        imgmsg = self._record_image()
        img = imgmsg2img(imgmsg)
        h, w = img.shape[:2]
        cv2.circle(img, (w/2, h/2), 4, (255, 0, 0), 2)
        self.display_image(_img2imgmsg(img))
        outpath = os.path.join(self._outpath, 'vs')
        rroi, _ = segment_area(imgmsg, outpath, th=250,
                               c_low=110, c_high=170, ff_connectivity=4,
                               a_low=700, a_high=10000)
        rroi, corners = rroi
        b = np.int0(corners)
        cv2.drawContours(img, [b], 0, (0, 255, 0), 2)
        cv2.circle(img, (int(rroi[0][0]), int(rroi[0][1])), 4,
                   (0, 255, 0), 2)
        self.display_image(_img2imgmsg(img))
        return rroi

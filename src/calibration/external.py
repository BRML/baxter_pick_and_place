# Copyright (c) 2016, BRML
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

import logging
import os

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

from axxa import tsai_lenz_89, inv_trafo_matrix
from hardware import Baxter, Kinect, img_to_imgmsg
from settings import settings
from settings.debug import topic_img4
from simulation import sim_or_real


class External(object):
    def __init__(self, arm, automatic, root_dir):
        """

        :param arm: The arm <'left', 'right'> to control.
        :param automatic:
        :param root_dir:
        """
        self._arm = arm
        self._automatic = automatic

        self.logger = logging.getLogger('cal_ext')

        self._robot = Baxter(sim=sim_or_real())
        self._robot.set_up(gripper=False)
        self._kinect = Kinect(root_dir=root_dir,
                              host=settings.elte_kinect_win_host)
        self._pub_vis = rospy.Publisher(topic_img4, Image,
                                        queue_size=10, latch=True)
        self._sink = os.path.join(root_dir, 'data', 'setup', 'external')
        if not os.path.exists(self._sink):
            self.logger.info('Creating folder {} to store calibration '
                             'images in.'.format(self._sink))
            os.makedirs(self._sink)

        if self._automatic:
            # self._lim = settings.task_space_limits_m
            self._lim = {
                'x_min': 0.9,
                'x_max': 0.9,
                'y_min': 0.2,
                'y_max': 0.2,
                'z_min': 0.1,
                'z_max': 0.1,
                'roll_min': -np.deg2rad(0.0),
                'roll_max': np.deg2rad(0.0),
                'pitch_min': np.pi/2 - np.deg2rad(50.0),
                'pitch_max': np.pi/2 + np.deg2rad(50.0),
                'yaw_min': -np.deg2rad(50.0),
                'yaw_max': np.deg2rad(50.0)
            }
            self.logger.info("Using task space limits")
            for c in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
                self.logger.info("{: .3f} <= {} <= {: .3f}".format(
                    self._lim['%s_min' % c], c, self._lim['%s_max' % c]))

        # This should go into the README:
        # Download the 4x11 asymmetric circle grid from
        # http://docs.opencv.org/2.4.13/_downloads/acircles_pattern.png
        self._patternsize = (4, 11)
        self._pattern = 0.017*np.array(
            [[x, y if x % 2 == 0 else y + 1, 0]
             for y in xrange(0, 8, 2)
             for x in xrange(11)], dtype=np.float32).T

    def _generate_random_poses(self, n):
        for _ in xrange(n):
            yield self._robot.sample_pose(lim=self._lim)

    def _modify_w2(self, pose):
        config = self._robot.ik(arm=self._arm, pose=pose)
        for key in config:
            if '_w2' in key:
                config[key] = 6.117*np.random.random_sample() - 3.059
                break
        return config

    def test_poses(self):
        i = 0
        for pose in self._generate_random_poses(n=10):
            if rospy.is_shutdown():
                break
            print 'Test pose', i
            i += 1
            try:
                config = self._modify_w2(pose=pose)
            except ValueError:
                print 'failed'
                continue
            self._robot.move_to_config(config=config)
            pose = self._robot.endpoint_pose(arm=self._arm)
            print "Pose is [", (" {: .3f}"*6).format(*pose), ']'

    def _generate_random_configs(self, n):
        for pose in self._generate_random_poses(n=n):
            try:
                yield self._modify_w2(pose=pose)
            except ValueError:
                yield None

    def test_configs(self):
        i = 0
        for config in self._generate_random_configs(n=10):
            if rospy.is_shutdown():
                break
            print 'Test config', i
            i += 1
            if config is None:
                print 'failed'
                continue
            self._robot.move_to_config(config=config)
            pose = self._robot.endpoint_pose(arm=self._arm)
            print "Pose is [" + (" {: .3f}"*6).format(*pose) + " ]"

    def _move_manual(self):
        self.logger.info("Manually move the calibration pattern.")
        c = 'a'
        while not c == 'r' and not rospy.is_shutdown():
            s = raw_input("Press [r] to record. ")
            c = s.lower()
        pose = self._robot.endpoint_pose(arm=self._arm)
        self.logger.debug("Pose is [" + (" {: .3f}"*6).format(*pose) + " ]")
        return True

    def _move_automatic(self):
        for config in self._generate_random_configs(n=1000):
            if config is not None:
                self._robot.move_to_config(config=config)
                pose = self._robot.endpoint_pose(arm=self._arm)
                self.logger.debug("Pose is [" + (" {: .3f}"*6).format(*pose) + " ]")
                return True
            return False

    def _move(self):
        if self._automatic:
            return self._move_automatic()
        else:
            return self._move_manual()

    def test_movement(self):
        mylist = list()
        while len(mylist) < 5:
            if rospy.is_shutdown():
                break
            ret = self._move()
            if ret:
                mylist.append(ret)
                print 'worked'
            else:
                print 'failed'
        print 'done'

    def manual_move_and_grab_data(self, arm):
        self.logger.info("Manually move the calibration pattern.")
        c = 'a'
        while not c == 's' and not rospy.is_shutdown():
            s = raw_input("Press [s] to record. ")
            c = s.lower()
        bttn = self._robot.hom_gripper_to_robot(arm=arm)
        color, _, _ = self._kinect.collect_data(color=True, depth=False,
                                                skeleton=False)
        return bttn, color

    def estimate_hand_trafo(self, n):
        """Estimate the transformation between Baxter's end effector and the
        origin of the calibration pattern mounted on it using the algorithm
        by Tsai and Lenz.

        :param n: The number of absolute end effector poses to use.
        :return: The homogeneous transform between TCP and the origin of the
            calibration pattern.
        """
        if n < 3:
            raise ValueError("At least 3 poses are needed to compute the "
                             "transformation!")
        btt = list()
        cto = list()
        pattern = self._pattern.T[:, np.newaxis, :]

        self.logger.info("Record %d absolute pose and pattern pairs." % n)
        while len(btt) < n and not rospy.is_shutdown():
            if self._move():
                rospy.sleep(1.0)
                bttn = self._robot.hom_gripper_to_robot(arm=self._arm)
                color, _, _ = self._kinect.collect_data(color=True)

                self._pub_vis.publish(img_to_imgmsg(img=color))
                patternfound, centers = cv2.findCirclesGridDefault(
                    image=color, patternSize=self._patternsize,
                    flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
                if patternfound == 0:
                    self.logger.debug("No pattern found!")
                    continue

                rot, trans, _ = cv2.solvePnPRansac(
                    objectPoints=pattern, imagePoints=centers,
                    cameraMatrix=self._kinect.color.camera_matrix,
                    distCoeffs=self._kinect.color.distortion_coeff,
                    flags=cv2.CV_ITERATIVE)
                rot, _ = cv2.Rodrigues(rot)
                cton = np.eye(4)
                cton[:-1, :-1] = rot
                cton[:-1, -1] = np.squeeze(trans)

                fname = os.path.join(self._sink, "1_tto_%d" % (len(btt) + 1))
                cv2.imwrite(fname + ".jpg", color)
                cv2.drawChessboardCorners(image=color,
                                          patternSize=self._patternsize,
                                          corners=centers,
                                          patternWasFound=patternfound)
                self._pub_vis.publish(img_to_imgmsg(img=color))
                cv2.imwrite(fname + "_det.jpg", color)

                btt.append(bttn)
                fname = os.path.join(self._sink, "1_btt_%d.npz" % len(btt))
                np.savez(fname, bttn=btt[-1])
                cto.append(cton)
                fname = os.path.join(self._sink, "1_cto_%d.npz" % len(cto))
                np.savez(fname, cton=cto[-1])

        self.logger.info("Compute %d relative transforms." % ((n**2 - n)/2))
        bttij = list()
        ctoij = list()
        for i in range(n):
            for j in range(i + 1, n):
                bttij.append(np.dot(inv_trafo_matrix(btt[j]), btt[i]))
                ctoij.append(np.dot(cto[j], inv_trafo_matrix(cto[i])))

        self.logger.info('Apply algorithm by Tsai and Lenz (1989).')
        tto = tsai_lenz_89(a=bttij, b=ctoij)
        fname = os.path.join(self._sink, "1_tto.npz")
        np.savez(fname, tto=tto)
        return tto

    def estimate_cam_trafo(self, n, tto):
        """Estimate the transformation between Baxter's base frame and the
        camera frame using singular value decomposition.

        :param n: The number of absolute end effector poses to use.
        :param tto: The homogeneous transform between TCP and the origin of
            the calibration pattern.
        :return: The homogeneous transform between Baxter's base frame and
            the camera frame.
        """
        bto = list()
        cto = list()

        self.logger.info("Record %d absolute point cloud pairs." % n)
        pattern = self._pattern.T[:, np.newaxis, :]
        hom_pattern = np.concatenate([self._pattern,
                                      np.ones((1, self._pattern.shape[1]))],
                                     axis=0)
        while len(bto) < n and not rospy.is_shutdown():
            if self._move():
                rospy.sleep(1.0)
                bttn = self._robot.hom_gripper_to_robot(arm=self._arm)
                color, _, _ = self._kinect.collect_data(color=True)

                self._pub_vis.publish(img_to_imgmsg(img=color))
                patternfound, centers = cv2.findCirclesGridDefault(
                    image=color, patternSize=self._patternsize,
                    flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
                if patternfound == 0:
                    self.logger.debug("No pattern found!")
                    continue

                rot, trans, _ = cv2.solvePnPRansac(
                    objectPoints=pattern, imagePoints=centers,
                    cameraMatrix=self._kinect.color.camera_matrix,
                    distCoeffs=self._kinect.color.distortion_coeff,
                    flags=cv2.CV_ITERATIVE)
                rot, _ = cv2.Rodrigues(rot)
                cton = np.eye(4)
                cton[:-1, :-1] = rot
                cton[:-1, -1] = np.squeeze(trans)

                fname = os.path.join(self._sink, "2_tto_%d" % (len(bto) + 1))
                cv2.imwrite(fname + ".jpg", color)
                cv2.drawChessboardCorners(image=color,
                                          patternSize=self._patternsize,
                                          corners=centers,
                                          patternWasFound=patternfound)
                self._pub_vis.publish(img_to_imgmsg(img=color))
                cv2.imwrite(fname + "_det.jpg", color)

                bto.append(np.dot(np.dot(bttn, tto), hom_pattern))
                fname = os.path.join(self._sink, "2_bto_%d.npz" % len(bto))
                np.savez(fname, bttn=bto[-1])
                cto.append(np.dot(cton, hom_pattern))
                fname = os.path.join(self._sink, "2_cto_%d.npz" % len(cto))
                np.savez(fname, cton=cto[-1])

        self.logger.info("Compute affine transformation from 3D point "
                         "correspondences.")
        # see http://stackoverflow.com/questions/15963960/opencv-2-4-estimateaffine3d-in-python
        # data of shape (Nx3)
        cto = np.array(cto).reshape((4, -1)).T[:, :-1]
        bto = np.array(bto).reshape((4, -1)).T[:, :-1]
        cto_mean = cto.mean(axis=0)
        bto_mean = bto.mean(axis=0)
        # compute covariance
        cov = np.dot((cto - cto_mean).T, bto - bto_mean)
        u, s, v = np.linalg.svd(cov)
        rot = np.dot(u, v.T)
        if np.linalg.det(rot) < 0:
            v[:, 2] = -v[:, 2]
            rot = np.dot(u, v.T)
        trans = bto_mean - np.dot(rot, cto_mean)
        trafo = np.eye(4)
        trafo[:-1, :-1] = rot
        trafo[:-1, -1] = trans
        fname = os.path.join(self._sink, "2_btc.npz")
        np.savez(fname, btc=trafo)
        return trafo

    def visual_test(self, tto, btc):
        """Visualize the result of the calibration.
        Compute the robot coordinates of the calibration pattern. Then project
        them to camera space and compute corresponding pixel coordinates. Draw
        them onto the image to see if the markings are reasonable close to the
        marks on the calibration pattern.

        :param tto: The homogeneous transform between TCP and the origin of
            the calibration pattern.
        :param btc: The homogeneous transform between Baxter's base frame and
            the camera frame.
        :return:
        """
        hom_pattern = np.concatenate([self._pattern,
                                      np.ones((1, self._pattern.shape[1]))],
                                     axis=0)
        tcp_pattern = np.dot(tto, hom_pattern)
        btc_inv = inv_trafo_matrix(trafo=btc)

        self.logger.info("Detect pattern and compare it with estimate.")
        patternfound = 0
        while patternfound == 0 and not rospy.is_shutdown():
            if self._move():
                rospy.sleep(1.0)
                btt = self._robot.hom_gripper_to_robot(arm=self._arm)
                color, _, _ = self._kinect.collect_data(color=True)

                self._pub_vis.publish(img_to_imgmsg(img=color))
                patternfound, centers = cv2.findCirclesGridDefault(
                    image=color, patternSize=self._patternsize,
                    flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
                if patternfound == 0:
                    self.logger.debug("No pattern found!")
                    continue

                fname = os.path.join(self._sink, "3_vis")
                cv2.imwrite(fname + ".jpg", color)
                cv2.drawChessboardCorners(image=color,
                                          patternSize=self._patternsize,
                                          corners=centers,
                                          patternWasFound=patternfound)
                self._pub_vis.publish(img_to_imgmsg(img=color))
                cv2.imwrite(fname + "_det.jpg", color)

                centers = centers[:, 0, :]
                rob_pattern = np.dot(btt, tcp_pattern)
                cam_pattern = np.dot(btc_inv, rob_pattern)
                pixels = np.zeros_like(centers)
                for i in xrange(cam_pattern.shape[1]):
                    coord = list(cam_pattern[:-1, i])
                    print i, coord,
                    pixels[i] = self._kinect.color.projection_camera_to_pixel(
                        position=coord)
                    # flip in y direction
                    pixels[i, 0] = color.shape[1] - pixels[i, 0]
                    print centers[i], pixels[i]
                    cv2.circle(color, tuple(int(x) for x in pixels[i]), 3,
                               [255, 0, 0] if i == 0 else [0, 255, 0], 2)
                self._pub_vis.publish(img_to_imgmsg(img=color))
                cv2.imwrite(fname + "_est.jpg", color)

                delta = centers - pixels
                self.logger.info("Offset in detected and estimated pixel "
                                 "coordinates:")
                self.logger.info("Mean:   {}".format(delta.mean(axis=0)))
                self.logger.info("Std:    {}".format(delta.std(axis=0)))
                self.logger.info("Median: {}".format(np.median(delta, axis=0)))


def perform_external_calibration(arm='left', n1=3, n2=1, automatic=True, root_dir=''):
    """Perform external camera calibration.

    :param arm: The arm <'left', 'right'> to control.
    :param n1: The number of absolute poses to use for hand trafo estimation.
    :param n2: The number of absolute poses to use for camera trafo estimation.
    :param root_dir: Where the baxter_pick_and_place package resides.
    :return:
    """
    pth = os.path.join(root_dir, 'data', 'setup', 'external')
    # pth = None

    def pp_flat(arr):
        """Pretty-print a flattened numpy array.

        :param arr: The array to flatten and format.
        :return: The flattened and formatted string representing the array.
        """
        return np.array_str(arr.flatten(), precision=2, max_line_width=150, suppress_small=True)

    tto_default = np.array([
        [0, -1, 0, 0.055],
        [-1, 0, 0, 0.075],
        [0, 0, -1, -0.08],
        [0, 0, 0, 1]
    ])
    btc_default = np.array([
        [0, 0, -1, 2.5],
        [-1, 0, 0, 0],
        [0, 1, 0, 0.35],
        [0, 0, 0, 1]
    ])

    cal = External(arm=arm, automatic=automatic, root_dir=root_dir)

    cal.logger.info("First, estimate trafo from pattern to TCP ...")
    # if pth is None:
    #     tto = cal.estimate_hand_trafo(n=n1)
    # else:
    with np.load(os.path.join(pth, "1_tto.npz")) as fp:
        tto = fp["tto"]
    cal.logger.info("Estimated hand trafo is {}.".format(pp_flat(tto)))
    cal.logger.info("Expected was similar to {}.".format(pp_flat(tto_default)))

    cal.logger.info("Second, estimate trafo from camera to robot base ...")
    # if pth is None:
    btc = cal.estimate_cam_trafo(n=n2, tto=tto)
    # else:
    #     with np.load(os.path.join(pth, "2_btc.npz")) as fp:
    #         btc = fp["btc"]
            # btc = inv_trafo_matrix(btc)
    cal.logger.info("Estimated camera trafo is {}.".format(pp_flat(btc)))
    cal.logger.info("Expected was similar to   {}.".format(pp_flat(btc_default)))

    cal.logger.info("Third, visualize estimate ...")
    cal.visual_test(tto=tto_default, btc=btc_default)
    cal._robot.clean_up(gripper=False)
    return btc

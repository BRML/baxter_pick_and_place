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
import numpy as np
import os

import cv2

import rospy

from axxa import tsai_lenz_89
from demo import settings
from hardware import Baxter, Kinect
from simulation import sim_or_real


class External(object):
    def __init__(self, root_dir):
        self.logger = logging.getLogger('cal_ext')
        self._robot = Baxter(sim=sim_or_real())
        self._kinect = Kinect(root_dir=root_dir,
                              host=settings.elte_kinect_win_host)
        self._sink = os.path.join(root_dir, 'data', 'setup', 'external')
        if not os.path.exists(self._sink):
            self.logger.info('Creating folder {} to store calibration '
                             'images in.'.format(self._sink))
            os.makedirs(self._sink)

        self._lim = settings.task_space_limits_m
        # TODO: tune rotation limits such that Kinect is able to detect pattern
        self._lim['roll_max'] = np.pi
        self._lim['roll_min'] = -np.pi
        self._lim['pitch_max'] = np.pi
        self._lim['pitch_min'] = -np.pi
        self._lim['yaw_max'] = np.pi
        self._lim['yaw_min'] = -np.pi

        # This should go into the README:
        # Download the 4x11 asymmetric circle grid from
        # http://docs.opencv.org/2.4.13/_downloads/acircles_pattern.png
        self._patternsize = (4, 11)
        self._pattern = 0.017*np.array(
            [[x, y if x % 2 == 0 else y + 1, 0]
             for y in xrange(0, 8, 2)
             for x in xrange(11)], dtype=np.float32).T

    def estimate_hand_trafo(self, n, arm):
        btt = list()
        cto = list()
        # record n absolute transformation pairs
        while len(btt) < n and not rospy.is_shutdown():
            self.logger.debug('try to record pose {} of {}.'.format(len(btt) + 1, n))
            pose = self._robot.sample_pose(lim=self._lim)
            try:
                self._robot.move_to_pose(arm=arm, pose=pose)
            except ValueError:
                continue
            bttn = self._robot.hom_gripper_to_robot(arm=arm)

            color, _, _ = self._kinect.collect_data(color=True, depth=False,
                                                    skeleton=False)
            patternfound, centers = cv2.findCirclesGridDefault(image=color,
                                                               patternSize=self._patternsize,
                                                               flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
            if patternfound == 0:
                self.logger.debug('no pattern found')
                continue
            pattern = self._pattern.T[:, np.newaxis, :]
            rot, trans, inliers = cv2.solvePnPRansac(objectPoints=pattern,
                                                     imagePoints=centers,
                                                     cameraMatrix=self._kinect.color.camera_matrix,
                                                     distCoeffs=self._kinect.color.distortion_coeff,
                                                     flags=cv2.CV_ITERATIVE)
            rot, jacobian = cv2.Rodrigues(rot)
            cton = np.eye(4)
            cton[:-1, :-1] = rot
            cton[:-1, -1] = np.squeeze(trans)

            fname = os.path.join(self._sink, '1_tto_{}'.format(len(btt) + 1))
            cv2.imwrite(fname + '.jpg', color)
            cv2.drawChessboardCorners(image=color, patternSize=self._patternsize,
                                      corners=centers, patternWasFound=patternfound)
            cv2.imwrite(fname + '_det.jpg', color)

            btt.append(bttn)
            fname = os.path.join(self._sink, '1_btt_{}.npz'.format(len(btt)))
            np.savez(fname, bttn=btt[-1])
            cto.append(cton)
            fname = os.path.join(self._sink, '1_cto_{}.npz'.format(len(cto)))
            np.savez(fname, cton=cto[-1])
        # compute sum_{i=1}^nn-1 relative transform pairs
        self.logger.debug('Compute {} relative transforms.'.format(sum(range(1, n))))
        bttij = list()
        ctoij = list()
        for i in range(n):
            for j in range(i + 1, n):
                bttij.append(np.dot(np.linalg.inv(btt[j]), btt[i]))
                ctoij.append(np.dot(np.linalg.inv(cto[j]), cto[i]))
        # apply tsai_lenz_89
        self.logger.debug('Apply algorithm by Tsai and Lenz (1989).')
        return tsai_lenz_89(a=bttij, b=ctoij)

    def estimate_cam_trafo(self, n, arm, tto):
        bto = list()
        cto = list()
        # record n absolute point cloud pairs
        hom_pattern = np.concatenate([self._pattern, np.zeros((1, self._pattern.shape[1]))], axis=0)
        while len(bto) < n and not rospy.is_shutdown():
            self.logger.debug('try to record pose {} of {}.'.format(len(bto) + 1, n))
            pose = self._robot.sample_pose(lim=self._lim)
            try:
                self._robot.move_to_pose(arm=arm, pose=pose)
            except ValueError:
                continue
            bttn = self._robot.hom_gripper_to_robot(arm=arm)

            color, _, _ = self._kinect.collect_data(color=True, depth=False,
                                                    skeleton=False)
            patternfound, centers = cv2.findCirclesGridDefault(image=color,
                                                               patternSize=self._patternsize,
                                                               flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
            if patternfound == 0:
                self.logger.debug('no pattern found')
                continue
            pattern = self._pattern.T[:, np.newaxis, :]
            rot, trans, inliers = cv2.solvePnPRansac(objectPoints=pattern,
                                                     imagePoints=centers,
                                                     cameraMatrix=self._kinect.color.camera_matrix,
                                                     distCoeffs=self._kinect.color.distortion_coeff,
                                                     flags=cv2.CV_ITERATIVE)
            rot, jacobian = cv2.Rodrigues(rot)
            cton = np.eye(4)
            cton[:-1, :-1] = rot
            cton[:-1, -1] = np.squeeze(trans)

            fname = os.path.join(self._sink, '2_tto_{}'.format(len(bto) + 1))
            cv2.imwrite(fname + '.jpg', color)
            cv2.drawChessboardCorners(image=color, patternSize=self._patternsize,
                                      corners=centers, patternWasFound=patternfound)
            cv2.imwrite(fname + '_det.jpg', color)

            bto.append(np.dot(np.dot(bttn, tto), hom_pattern))
            fname = os.path.join(self._sink, '2_bto_{}.npz'.format(len(bto)))
            np.savez(fname, bttn=bto[-1])
            cto.append(np.dot(cton, hom_pattern))
            fname = os.path.join(self._sink, '2_cto_{}.npz'.format(len(cto)))
            np.savez(fname, cton=cto[-1])
        # compute affine transformation from 3D point correspondences
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
        return trafo


def perform_external_calibration(arm='left', n1=3, n2=1, root_dir=''):
    cal = External(root_dir=root_dir)
    cal.logger.info('First, estimate trafo from pattern to TCP ...')
    tto = cal.estimate_hand_trafo(n=n1, arm=arm)
    cal.logger.info('Second, estimate trafo from camera to robot base ...')
    btc = cal.estimate_cam_trafo(n=n2, arm=arm, tto=tto)
    return btc

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

from hardware import img_to_imgmsg
from vision import mask_to_rroi, draw_rroi, draw_detection


class Servoing(object):
    def __init__(self, robot, detection, segmentation, pub_vis, object_size, tolerance):
        """Base class for visual servoing. Can be used to position the end
        effector directly over the requested object.
        Note: Assumes that the end effector is restricted to pointing along
            the vertical (z) axis. That is, the object position in the x-y
            plane and the distance to the object along z direction are
            estimated.

        :param robot: A robot abstraction module instance.
        :param segmentation: An object segmentation module instance.
        :param pub_vis: A ROS publisher to publish visualization images with.
        :param object_size: The measured length of the longer dimension of
            each object (in the x-y plane) in meters.
        :param tolerance: The position error tolerance in meters.
        """
        self._robot = robot
        self._detection = detection
        self._segmentation = segmentation
        self._pub_vis = pub_vis
        self._object_size_meters = object_size
        self._tolerance = tolerance

        self._logger = logging.getLogger('main.servo')

    def _find_rotated_enclosing_rect(self, image, object_id):
        """Find the rectangle with arbitrary orientation that encloses the
        segmented object in the given image with minimum area.
        Note: The passed image is being modified and published on the
            visualization image topic!

        :param image: An image (numpy array) of shape (height, width, 3).
        :param object_id: The object identifier.
        :return: The rotated rectangle enclosing the segmented object,
            given by ((cx, cy), (w, h), alpha).
        :raise: ValueError if the given object could not be segmented.
        """
        # first, detect object in image
        if object_id == 'hand':
            det = self._detection.detect_best(image=image, threshold=0.5)
        else:
            det = self._detection.detect_object(image=image,
                                                object_id=object_id,
                                                threshold=0.5)
        img_copy = np.copy(image)
        draw_detection(image=img_copy, detections=det)
        self._pub_vis.publish(img_to_imgmsg(img=img_copy))

        # second, segment object within bounding box
        if det['box'] is not None:
            xul, yul, xlr, ylr = [int(round(x)) for x in det['box']]
            selection = np.copy(image[yul:ylr, xul:xlr])
            self._pub_vis.publish(img_to_imgmsg(img=selection))
            seg = self._segmentation.detect_best(image=image[yul:ylr, xul:xlr],
                                                 threshold=0.8)
            self._pub_vis.publish(img_to_imgmsg(img=seg['mask']))

            handstring = ' in hand' if object_id is 'hand' else ''
            if seg['mask'] is not None:
                self._logger.info("Segmented {}{}.".format(seg['id'], handstring))
                # place segmentation in appropriate place in image
                seg['box'] += np.array([xul, yul, xul, yul])
                mask = np.zeros(shape=image.shape[:2], dtype=np.uint8)
                h, w = seg['mask'].shape[:2]
                mask[yul:yul+h, xul:xul+w] = seg['mask']
                seg['mask'] = mask
                seg['id'] = det['id']
                seg['score'] = det['score']
                draw_detection(image=image, detections=seg)
                self._pub_vis.publish(img_to_imgmsg(img=image))
                rroi = mask_to_rroi(mask=seg['mask'])
            else:
                raise ValueError("Segmentation of {}{} failed!".format(seg['id'],
                                                                       handstring))
        else:
            raise ValueError("Detection of {} failed!".format(object_id))
        draw_rroi(image=image, rroi=rroi)
        self._pub_vis.publish(img_to_imgmsg(img=image))
        return rroi

    def estimate_distance(self, object_id, rroi, arm):
        """Estimate the distance to the object.

        :param object_id: The object identifier of the object to estimate the
            distance to.
        :param rroi: The rotated rectangle enclosing the segmented object,
            given by ((cx, cy), (w, h), alpha).
        :param arm: The arm <'left', 'right'> to control.
        :return: The approximate distance from the gripper to the object.
        """
        raise NotImplementedError()

    def _pixel_to_camera_factor(self, object_id, rroi, arm):
        """Scale factor mapping from pixels to meters at the current distance.

        :param object_id: The object identifier of the object to estimate the
            distance to.
        :param rroi: The rotated rectangle enclosing the segmented object,
            given by ((cx, cy), (w, h), alpha).
        :param arm: The arm <'left', 'right'> to control.
        :return: The scale factor.
        """
        return (
            self._robot.cameras[arm].meters_per_pixel *
            self.estimate_distance(arm=arm, rroi=rroi, object_id=object_id)
        )

    def _error(self, image_size, object_id, rroi, arm):
        """Convert the offset between the object's center and the image
        center in pixels into the corresponding offset in meters.

        :param image_size: The height and width of the image (h, w).
        :param object_id: The object identifier of the object to estimate the
            distance to.
        :param rroi: The rotated rectangle enclosing the segmented object,
            given by ((cx, cy), (w, h), alpha).
        :param arm: The arm <'left', 'right'> to control.
        :return: The position error in meters.
        """
        h, w = image_size
        pixel_delta = [a - b for a, b in zip((w//2, h//2), rroi[0])]
        pixel_error = np.sqrt(np.dot(pixel_delta, pixel_delta))
        p2c_factor = self._pixel_to_camera_factor(object_id=object_id,
                                                  rroi=rroi, arm=arm)
        camera_error = pixel_error*p2c_factor
        if camera_error < 0.0:
            self._logger.error("Distances are < 0 m! Is your table height "
                               "estimate correct?")
        return pixel_error*p2c_factor

    def _iterate(self, arm, object_id, rroi):
        """Perform one update of the position of the end effector relative to
        the given object.

        :param arm: The arm <'left', 'right'> to control.
        :param object_id: The object identifier of the object to estimate the
            distance to.
        :param rroi: The rotated rectangle enclosing the segmented object,
            given by ((cx, cy), (w, h), alpha).
        :return: A tuple containing
            - the rotated rectangle enclosing the segmented object, given by
                ((cx, cy), (w, h), alpha) and
            - the position error in meters.
        """
        kp = 0.7  # proportional control parameter

        img = self._robot.cameras[arm].collect_image()
        self._pub_vis.publish(img_to_imgmsg(img=img))
        camera_error = self._error(image_size=img.shape[:2],
                                   object_id=object_id, rroi=rroi, arm=arm)
        if camera_error > self._tolerance:
            p2c_factor = self._pixel_to_camera_factor(object_id=object_id,
                                                      rroi=rroi, arm=arm)
            h, w = img.shape[:2]
            # dx = a*(h/2 - cy)
            # dy = a*(w/2 - cx)
            dy, dx = [(a - b)*p2c_factor*kp
                      for a, b in zip((w//2, h//2), rroi[0])]
            # dz = -estimated_distance/2
            dz = -self.estimate_distance(arm=arm, rroi=rroi,
                                         object_id=object_id)/2.0
            self._logger.debug("Computed position update is ({: .3f}, "
                               "{: .3f}, {: .3f}).".format(dx, dy, dz))
            pose = self._robot.endpoint_pose(arm=arm)
            # TODO: verify this update works as expected
            pose = [a + b for a, b in zip(pose, [dx, dy, dz, 0, 0, 0])]# rroi[2]])]
            if pose[2] < self._robot.z_table:
                pose[2] = self._robot.z_table
            try:
                cfg = self._robot.ik(arm=arm, pose=pose)
                self._robot.move_to_config(config=cfg)
                rroi = self._find_rotated_enclosing_rect(image=img,
                                                         object_id=object_id)
                camera_error = self._error(image_size=img.shape[:2],
                                           object_id=object_id, rroi=rroi,
                                           arm=arm)
            except ValueError as e:
                self._logger.error(e)
        return rroi, camera_error

    def servo(self, arm, object_id):
        """Apply visual servoing to position the end effector over the given
        object.

        :param arm: The arm <'left', 'right'> to control.
        :param object_id: The object identifier of the object to servo to.
        :return: boolean success value.
        """
        img = self._robot.cameras[arm].collect_image()
        self._pub_vis.publish(img_to_imgmsg(img=img))
        try:
            rroi = self._find_rotated_enclosing_rect(image=img,
                                                     object_id=object_id)
        except ValueError as e:
            self._logger.error(e)
            return False
        camera_error = 2*self._tolerance
        it = 1
        while camera_error > self._tolerance:
            rroi, camera_error = self._iterate(arm=arm, object_id=object_id,
                                               rroi=rroi)
            self._logger.debug("Iteration {} finished. Error is {} m {} {} "
                               "m.".format(it, camera_error,
                                           '>' if camera_error > self._tolerance else '<=',
                                           self._tolerance))
            it += 1
        # prepare to grasp
        pose = self._robot.endpoint_pose(arm=arm)
        pose[2] = self._robot.z_table + 0.01
        try:
            cfg = self._robot.ik(arm=arm, pose=pose)
            self._robot.move_to_config(config=cfg)
        except ValueError as e:
            self._logger.error(e)
            return False
        return True

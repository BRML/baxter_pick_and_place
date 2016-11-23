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

import rospy

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
        self._tsleep = 0.33

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
        rospy.sleep(self._tsleep)

        # second, segment object within bounding box
        if det['box'] is not None:
            xul, yul, xlr, ylr = [int(round(x)) for x in det['box']]
            # selection = np.copy(image[yul:ylr, xul:xlr])
            # self._pub_vis.publish(img_to_imgmsg(img=selection))
            # rospy.sleep(self._tsleep)
            seg = self._segmentation.detect_best(image=image[yul:ylr, xul:xlr],
                                                 threshold=0.8)
            # self._pub_vis.publish(img_to_imgmsg(img=seg['mask']))
            # rospy.sleep(self._tsleep)

            handstring = ' in hand' if object_id is 'hand' else ''
            if seg['mask'] is not None:
                self._logger.debug("Segmented {}{}.".format(seg['id'], handstring))
                # place segmentation in appropriate place in image
                seg['box'] += np.array([xul, yul, xul, yul])
                mask = np.zeros(shape=image.shape[:2], dtype=np.uint8)
                h, w = seg['mask'].shape[:2]
                mask[yul:yul+h, xul:xul+w] = seg['mask']
                seg['mask'] = mask
                seg['id'] = det['id']
                seg['score'] = det['score']
                draw_detection(image=image, detections=seg)
                rroi = mask_to_rroi(mask=seg['mask'])
            else:
                raise ValueError("Segmentation of {}{} failed!".format(seg['id'],
                                                                       handstring))
        else:
            raise ValueError("Detection of {} failed!".format(object_id))
        draw_rroi(image=image, rroi=rroi)
        self._pub_vis.publish(img_to_imgmsg(img=image))
        rospy.sleep(self._tsleep)
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

    def update_pose(self, arm, object_id, rroi, img_size):
        """Update the end effector pose according to the estimated pose of
        the detected object.

        :param arm: The arm <'left', 'right'> to control.
        :param object_id: The object identifier of the object to estimate the
            distance to.
        :param rroi: The rotated rectangle enclosing the segmented object,
            given by ((cx, cy), (w, h), alpha).
        :param img_size: The size of the image in which the object was
            detected.
        :return:
        """
        kp = 0.9  # proportional control parameter

        # delta in pixel space
        h, w = img_size
        d_pixel = [a - b for a, b in zip((w//2, h//2), rroi[0])]
        # delta in camera space
        p2c_factor = self._pixel_to_camera_factor(object_id=object_id,
                                                  rroi=rroi, arm=arm)
        d_cam = [x*p2c_factor for x in d_pixel]
        # delta in robot space
        # assuming that orientation of end effector is perpendicular to table
        rot = self._robot.hom_camera_to_robot(arm=arm)[:2, :2]
        d_rob = np.dot(rot, d_cam)
        # update
        dx, dy = [-x*kp for x in d_rob]
        dz = -self.estimate_distance(arm=arm, rroi=rroi,
                                     object_id=object_id)/3.0
        self._logger.debug("Computed position update is ({: .3f}, "
                           "{: .3f}, {: .3f}) m.".format(dx, dy, dz))

        pose = self._robot.endpoint_pose(arm=arm)
        pose = [a + b for a, b in zip(pose, [dx, dy, dz,
                                             0, 0, -np.deg2rad(rroi[2])])]
        if pose[2] < self._robot.z_table:
            pose[2] = self._robot.z_table
        cfg = self._robot.ik(arm=arm, pose=pose)
        self._robot.move_to_config(config=cfg)

    def correct_height(self, arm):
        """Make sure the gripper height is appropriate before attempting to
        grasp the object.

        :param arm: The arm <'left', 'right'> to control.
        :return: A boolean success value.
        """
        raise NotImplementedError()

    def servo(self, arm, object_id):
        """Apply visual servoing to position the end effector over the given
        object.

        :param arm: The arm <'left', 'right'> to control.
        :param object_id: The object identifier of the object to servo to.
        :return: A boolean success value.
        """
        it = 0
        while not rospy.is_shutdown():
            img = self._robot.cameras[arm].collect_image()
            try:
                rroi = self._find_rotated_enclosing_rect(image=img,
                                                         object_id=object_id)
            except ValueError as e:
                self._logger.error(e)
                return False
            camera_error = self._error(image_size=img.shape[:2],
                                       object_id=object_id, rroi=rroi,
                                       arm=arm)
            accept = camera_error <= self._tolerance
            self._logger.info("In iteration {}, error is {:.4f} m {} {:.4f} "
                              "m.".format(it, camera_error,
                                          '<=' if accept else '>',
                                          self._tolerance))
            if accept:
                break
            try:
                self.update_pose(arm=arm, object_id=object_id, rroi=rroi, img_size=img.shape[:2])
            except ValueError as e:
                self._logger.error(e)
                return False
            it += 1
        # make sure we are in appropriate height
        return self.correct_height(arm=arm)

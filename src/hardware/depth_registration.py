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

# DISCLAIMER: This part of the software has been adapted from
# https://github.com/code-iai/iai_kinect2/tree/master/kinect2_registration.

import numpy as np

import cv2


class DepthRegistration(object):
    def __init__(self, cam_mat_registered, size_registered,
                 cam_mat_depth, size_depth, distortion_depth,
                 rotation, translation, z_near=0.5, z_far=12.0):
        self._cam_mat_registered = cam_mat_registered
        self._size_registered = size_registered
        self._cam_mat_depth = cam_mat_depth
        self._size_depth = size_depth

        self._proj = np.eye(4)
        self._proj[:3, :3] = rotation
        self._proj[:3, -1] = translation

        self._z_near = z_near
        self._z_far = z_far

        map_x, map_y = cv2.initUndistortRectifyMap(
            cameraMatrix=self._cam_mat_depth, distCoeffs=distortion_depth,
            R=np.eye(3), newCameraMatrix=self._cam_mat_registered,
            size=self._size_registered[::-1], m1type=cv2.CV_32FC1)
        self._map_x = map_x
        self._map_y = map_y
        # print 'map x', self._map_x.shape, self._map_x.dtype, self._map_x.min(), self._map_x.max()
        # print 'map y', self._map_y.shape, self._map_y.dtype, self._map_y.min(), self._map_y.max()
        # cv2.imshow('map x', self._map_x/self._map_x.max())
        # cv2.imshow('map y', self._map_y/self._map_y.max())
        # cv2.waitKey(33)

        self._fx = self._cam_mat_registered[0, 0]
        self._fy = self._cam_mat_registered[1, 1]
        self._cx = self._cam_mat_registered[0, 2] + 0.5
        self._cy = self._cam_mat_registered[1, 2] + 0.5

        self._lookup_x = None
        self._lookup_y = None
        self._create_lookup()

    def _create_lookup(self):
        fx = 1.0/self._cam_mat_registered[0, 0]
        fy = 1.0/self._cam_mat_registered[1, 1]
        cx = self._cam_mat_registered[0, 2]
        cy = self._cam_mat_registered[1, 2]

        lookup_y = np.arange(self._size_registered[0], dtype=np.float64)
        self._lookup_y = (lookup_y - cy)*fy

        lookup_x = np.arange(self._size_registered[1], dtype=np.float64)
        self._lookup_x = (lookup_x - cx)*fx

    @staticmethod
    def _interpolate(img, x, y):
        (x_l, x_h), (y_l, y_h) = [(int(np.floor(v)), int(np.ceil(v)))
                                  for v in [x, y]]
        if x_l < 0 or y_l < 0 or x_h >= img.shape[1] or y_h >= img.shape[0]:
            return 0

        depth = np.array([img[y_l, x_l], img[y_l, x_h], img[y_h, x_l], img[y_h, x_h]])
        gtz = depth > 0
        if gtz.sum() < 3:
            return 0
        avg = depth.mean()
        thres = 0.01*avg
        stt = np.abs(depth - avg) < thres
        if stt.sum() < 3:
            return 0

        dist_xl = x - x_l
        dist_xh = 1.0 - dist_xl
        dist_yl = y - y_l
        dist_yh = 1.0 - dist_yl

        dist_x = np.array([dist_xl, dist_xh, dist_xl, dist_xh])**2
        dist_y = np.array([dist_yl, dist_yl, dist_yh, dist_yh])**2
        f = np.where(stt, np.sqrt(2) - np.sqrt(dist_x + dist_y), 0)
        return np.dot(depth, f)/f.sum() + 0.5

        # p_lt = img[y_l, x_l]
        # p_rt = img[y_l, x_h]
        # p_lb = img[y_h, x_l]
        # p_rb = img[y_h, x_h]
        # v_lt = p_lt > 0
        # v_rt = p_rt > 0
        # v_lb = p_lb > 0
        # v_rb = p_rb > 0
        # count = sum([1 if v else 0 for v in [v_lt, v_rt, v_lb, v_rb]])
        # if count < 3:
        #     return 0
        #
        # avg = (p_lt + p_rt + p_lb + p_rb)/count
        # thres = 0.01*avg
        # v_lt, v_rt, v_lb, v_rb = [abs(x - avg) < thres
        #                           for x in [p_lt, p_rt, p_lb, p_rb]]
        # count = sum([1 if v else 0 for v in [v_lt, v_rt, v_lb, v_rb]])
        # if count < 3:
        #     return 0
        #
        # dist_xl = x - x_l
        # dist_xh = 1.0 - dist_xl
        # dist_yl = y - y_l
        # dist_yh = 1.0 - dist_yl
        # dist_xl, dist_xh, dist_yl, dist_yh = \
        #     [x**2 for x in [dist_xl, dist_xh, dist_yl, dist_yh]]
        # tmp = np.sqrt(2.0)
        # f_lt = tmp - np.sqrt(dist_xl + dist_yl) if v_lt else 0
        # f_rt = tmp - np.sqrt(dist_xh + dist_yl) if v_rt else 0
        # f_lb = tmp - np.sqrt(dist_xl + dist_yh) if v_lb else 0
        # f_rb = tmp - np.sqrt(dist_xh + dist_yh) if v_rb else 0
        # denominator = f_lt + f_rt + f_lb + f_rb
        # return ((p_lt*f_lt + p_rt*f_rt + p_lb*f_lb + p_rb*f_rb)/denominator) + 0.5

    def _remap_depth(self, depth):
        # map given depth image to a rescaled version;
        # interpolate the new depth values
        # scaled = cv2.resize(depth, self._size_registered[::-1])
        scaled = np.empty(self._size_registered, dtype=np.uint16)
        for row in xrange(scaled.shape[0]):
            for col in xrange(scaled.shape[1]):
                scaled[row, col] = self._interpolate(img=depth,
                                                     x=self._map_x[row, col],
                                                     y=self._map_y[row, col])
        return scaled

    def _project_depth(self, scaled):
        registered = np.zeros(self._size_registered, dtype=np.uint16)
        for row in xrange(registered.shape[0]):
            y = self._lookup_y[row]
            for col in xrange(registered.shape[1]):
                x = self._lookup_x[col]
                depth_value = scaled[row, col]/1000.0
                if depth_value < self._z_near or depth_value > self._z_far:
                    continue
                point_d = np.array([x*depth_value, y*depth_value, depth_value, 1])
                point_p = np.dot(self._proj, point_d)

                z = point_p[2]
                inv_z = 1.0/z
                x_p = self._fx*point_p[0]*inv_z + self._cx
                y_p = self._fy*point_p[1]*inv_z + self._cy

                if (0 <= x_p < self._size_registered[1] and
                        0 <= y_p < self._size_registered[0]):
                    z_16 = z*1000
                    if registered[y_p, x_p] == 0 or z_16 < registered[y_p, x_p]:
                        registered[y_p, x_p] = z_16
        return registered

    def register_depth(self, depth):
        # registered: np.zeros((h, w), dtype=np.uint16)
        # modifies the given registered image in place
        # scaled = cv2.remap(src=depth, map1=self._map_y, map2=self._map_x, interpolation=cv2.INTER_CUBIC)
        scaled = self._remap_depth(depth=depth)
        print 'scaled:', scaled.shape, scaled.dtype, scaled.min(), scaled.max()
        cv2.imshow('scaled', scaled)
        cv2.waitKey(33)
        registered = self._project_depth(scaled=scaled)
        print 'after, registered:', registered.shape, registered.dtype, registered.min(), registered.max()
        return registered


if __name__ == '__main__':
    import time

    reg = np.array([1.0556223863649359e+03, 0., 9.5417339830340200e+02, 0.,
                    1.0550357564529666e+03, 5.3003553108782205e+02, 0., 0., 1.]).reshape((3, 3))/2.0
    sreg = (1080, 1920)
    dep = np.array([3.6441717090792264e+02, 0., 2.4976446015643833e+02, 0.,
                    3.6432176899325952e+02, 2.0868139387945234e+02, 0., 0., 1.]).reshape((3, 3))
    sdep = (424, 512)
    dist_dep = np.array([1.0814451687857619e-01, -3.4092997612470310e-01,
                         1.8635310483513959e-03, 8.2329335500367794e-04,
                         1.7735068907380566e-01])
    rot = np.array([9.9997280609829253e-01, 4.6300753995816308e-03,
                    -5.7401625151890772e-03, -4.6266789480935190e-03,
                    9.9998911394748591e-01, 6.0483752638133749e-04,
                    5.7429004708301392e-03, -5.7826318942143142e-04,
                    9.9998334221419205e-01]).reshape((3, 3))
    trans = np.array([-4.9143118506968717e-02, 3.1501251792413907e-05, 1.5600404762627028e-03])
    dr = DepthRegistration(reg, sreg, dep, sdep,
                           dist_dep, rot, trans,
                           0.5, 12.0)

    with np.load('/home/mludersdorfer/software/ws_baxter_pnp/src/baxter_pick_and_place/data/rgbd.npz') as fp:
        c = fp['color']
        d = fp['depth']
    print 'outside, depth:', d.shape, d.dtype, d.min(), d.max()
    cv2.imshow('color', c)
    cv2.imshow('depth', d)
    cv2.waitKey(33)
    start = time.time()
    r = dr.register_depth(d)
    print 'outside, after, registered:', r.shape, r.dtype, r.min(), r.max()
    # scaled = cv2.resize(src=d, dsize=c.shape[:2], interpolation=cv2.INTER_CUBIC)
    # reg_fx = reg[0, 0]
    # reg_fy = reg[1, 1]
    # reg_cx = reg[0, 2]
    # reg_cy = reg[1, 2]
    #
    # px, py = 123, 123
    # z_3d = scaled[int(py), int(px)]
    # x_3d = (px - reg_cx)/reg_fx*z_3d
    # y_3d = (py - reg_cy)/reg_fy*z_3d
    # print (x_3d, y_3d, z_3d)
    print 'took {:.3f} s to compute.'.format(time.time() - start)
    cv2.imshow('registered', r)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

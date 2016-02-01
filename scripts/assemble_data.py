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

import csv
import cv2
import errno
import glob
import gzip
import numpy as np
import os
import pandas as pd
import time

from baxter_pick_and_place.rand import rand_x_digit_num


def write_objects(objects, filename):
    """ Write list of objects to file.
    :param objects: list of objects
    :param filename: name of file to write to
    """
    with open(filename, 'w') as thefile:
        for obj in objects:
            thefile.write("%s\n" % obj)


def read_objects(filename):
    """ Read list of objects from file.
    :param filename: name of file to read from
    :return: list of objects
    """
    with open(filename, 'r') as thefile:
        objects = thefile.read().splitlines()
    return objects


def assemble_data(objects, num_patches, images, base_dirname, image_dirname,
                  csv_filename, overwrite=True):
    """ Generate data set and data set description with a given number of
    images from a set of base images (background plus foreground objects).
    :param objects: list of objects to use
    :param num_patches: maximum number of objects to place on each image
    :param images: number of images to generate
    :param base_dirname: directory to read base images from
    :param image_dirname: directory to write generated images to
    :param csv_filename: CSV file to write generated images' parameter to
    :param overwrite: Whether to overwrite existing CSV file
    """
    csv_header = ['image_name', 'image_filename', 'patch_parameters', 'labels']
    parameters = dict()
    parameters['max_nr_patches'] = num_patches
    # table workspace in pixel coordinates (background images)
    parameters['x_min'] = 400
    parameters['x_max'] = 1020
    parameters['y_min'] = 310
    parameters['y_max'] = 670

    # prepare lists of available base images (plus labels) for image generation
    files = glob.glob(os.path.join(base_dirname, '*.jpg'))
    background_list = [f for f in files if 'background' in f]
    object_list = list()
    for f in files:
        for idx, obj in enumerate(objects):
            if obj in f:
                object_list.append((f, idx))

    # write header to CSV file
    if overwrite:
        with gzip.open(csv_filename, 'w') as fp:
            csv_writer = csv.writer(fp)
            csv_writer.writerow(csv_header)

        # overwriting, therefore remove old images
        old_images = glob.glob(os.path.join(image_dirname, '*.jpg'))
        for oi in old_images:
            try:
                os.remove(oi)
            except OSError as e:
                if e.errno != errno.ENOENT:  # 'no such file or directory'
                    raise  # re-raise if different error occurred

    start = time.time()
    # generate required number of images with custom properties
    # and write them to CSV file
    for idx in range(images):
        d = _assemble_image(background_list, object_list, parameters,
                            image_dirname)
        if len(d) != len(csv_header):
            raise ValueError('Mismatch between image description data and -header!')
        with gzip.open(csv_filename, 'a') as fp:
            csv_writer = csv.writer(fp)
            csv_writer.writerow(d)

    print 'Wrote %i image(s) in %.3fs.' % (images, time.time() - start)
    cv2.destroyAllWindows()


def _assemble_image(bg_list, fg_list, parameters, image_dirname):
    """ Assemble one image for the synthetic data set.
    :param bg_list: list of background image files
    :param fg_list: list of foreground image files
    :param parameters: dictionary of parameters for random foreground
    placement
    :param image_dirname: directory to save generated images to
    :return: image-description list [image name, image filename, [list of
    foreground placement parameters], [list of corresponding labels]]
    """
    image_name = rand_x_digit_num(12)
    image_filename = os.path.join(image_dirname, image_name + '.jpg')

    # sample image parameters
    # sample background
    background_file = bg_list[np.random.randint(len(bg_list))]
    img = cv2.imread(background_file)
    cv2.imshow('image', img)

    # sample number of patches to overlay
    nr_patches = np.random.randint(parameters['max_nr_patches'])
    patches = [fg_list[np.random.randint(len(fg_list))]
               for _ in range(nr_patches)]
    patch_parameters = list()
    labels = list()
    for p, lbl in patches:
        patch = cv2.imread(p)
        h, w = patch.shape[:2]
        if w > 360 or h > 360:
            raise ValueError('Something is wrong with patch %s!' % p)

        # random rotation of patch
        alpha = 180*(np.random.random() - 0.5)
        patch = _aug_rotate(patch, alpha)
        nh, nw = patch.shape[:2]
        if not (parameters['x_max'] - nw > parameters['x_min'] and
                parameters['y_max'] - nh > parameters['y_min']):
            continue
            # raise ValueError("Rotated patch too large for background table!")

        # random translation of patch
        # TODO: make sure that patches do not overlay???
        tx = np.random.randint(parameters['x_min'], parameters['x_max'] - nw)
        ty = np.random.randint(parameters['y_min'], parameters['y_max'] - nh)
        img[ty:ty + nh, tx:tx + nw] = patch

        patch_parameters.append((p, (tx, ty, w, h, alpha)))
        labels.append(lbl)

    cv2.imshow('image', img)
    cv2.waitKey(0)
    cv2.imwrite(image_filename, img)
    return [image_name, image_filename, patch_parameters, labels]


def _aug_rotate(image, angle):
    """ Rotate image by an angle in degrees.
    :param image: the image to rotate
    :param angle: the angle by which to rotate (in degrees). Positive values
    rotate clockwise
    :return: the rotated image
    """
    h, w = image.shape[:2]
    # compute size after rotation
    nw, nh = _newdim(w, h, angle)
    # shift original image to center of new image
    tx = .5*(nw - w)
    ty = .5*(nh - h)
    trans = np.array([[1, 0, tx], [0, 1, ty]])
    image = cv2.warpAffine(image, trans, (nw, nh))
    # rotate image by given angle
    rot = cv2.getRotationMatrix2D((.5*nw, .5*nh), angle, 1.0)
    # TODO: border mode for affine transform
    return cv2.warpAffine(image, rot, (nw, nh))


def _newdim(w, h, angle):
        """ Compute new dimension of patch after rotation.
        :param w: original patch width
        :param h: original patch height
        :param angle: rotation angle in degrees
        :return: tuple (width, height)
        """
        corners = .5*np.array([[-w, -h], [w, -h], [-w, h], [w, h]])
        rot = cv2.getRotationMatrix2D((0, 0), angle, 1.0)
        corners_hat = np.dot(rot[:, :2], corners.transpose())
        return tuple([int(a) for a in np.ceil(corners_hat.ptp(axis=1))])


def create_split(csv_filename, data_dirname, labels, images, split=(.7, .3)):
    """ Create a (train/test/validation)-data set from the data set definition.
    :param csv_filename: source file containing image descriptions
    :param data_dirname: directory to write data sets to
    :param labels: sample only data from the first number of labels
    :param images: number of images to use [-1 for all]
    :param split: test/train/val split to use [default is (.7, .3)]
    :return: boolean flag
    """
    if not 0 < len(split) < 4:
        return False

    # read data, shuffle order and subsample
    df = pd.read_csv(csv_filename, index_col=0, compression='gzip')
    df = df.iloc[np.random.permutation(df.shape[0])]
    if labels > 0:
        df = df.loc[df['label'] < labels]
    if images > df.shape[0] or images == -1:
        images = df.shape[0]
    # number of images per split
    nr = [int(np.floor(images*s)) for s in split]
    nr = [np.sum(nr[:s]) for s in range(1, len(nr) + 1)]
    split = ['train', 'test', 'val']
    for idx in range(len(nr)):
        if idx == 0:
            df[split[idx]] = df.iloc[0:nr[idx]]
        else:
            df[split[idx]] = df.iloc[nr[idx - 1]:nr[idx]]

    # write out training and testing file lists
    for idx in range(len(nr)):
        filename = os.path.join(data_dirname, '%s.txt' % split[idx])
        df[split[idx]][['image_filename', 'label']].to_csv(filename, sep=' ',
                                                           header=None,
                                                           index=None)
    print 'Wrote data sets for %i images.' % np.sum(nr)
    return True


def main():
    """ Assemble data for the 'synthetic demo data' set.

    Create a synthetic data set by placing object patches on top of background
    images. Apply a number of transformations to the images in order to
    'augment' the data set (cf. data augmentation).
    """
    script_dirname = os.path.abspath(os.path.dirname(__file__))
    base_dirname = os.path.abspath(os.path.join(script_dirname, '..', 'data',
                                                'sdd'))
    data_dirname = os.path.abspath(os.path.join(script_dirname, '..', 'data',
                                                'synthetic_demo_data'))
    if not os.path.exists(data_dirname):
        os.makedirs(data_dirname)
    image_dirname = os.path.join(data_dirname, 'images')
    if not os.path.exists(image_dirname):
        os.makedirs(image_dirname)
    csv_filename = os.path.join(data_dirname, 'synthetic_demo_data.csv.gz')

    # write object names (label strings) to file,
    # their order in the file gives the label int's
    objects = ['bin', 'duplo_brick', 'extra_mints', 'glue_stick',
               'golf_ball', 'pen']
    # fn = os.path.join(data_dirname, 'object_names.txt')
    # write_objects(filename=fn)

    assemble_data(objects, num_patches=5, images=1, base_dirname=base_dirname,
                  image_dirname=image_dirname, csv_filename=csv_filename,
                  overwrite=True)
    # create_split(csv_filename, data_dirname, 0, 10)

if __name__ == '__main__':
    main()

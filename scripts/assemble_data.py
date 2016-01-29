import csv
import glob
import gzip
import numpy as np
import os
import pandas as pd
import time


objects = ['bin', 'duplo_brick', 'extra_mints', 'glue_stick', 'golf_ball', 'pen']


def write_objects(filename):
    """ Write list of objects to file.
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


def assemble_data(objects, images, base_dirname, image_dirname, csv_filename,
                  overwrite=True):
    """ Generate data set and data set description with a given number of
    images from a set of base images (background plus foreground objects).
    :param objects: list of objects to use
    :param images: number of images to generate
    :param base_dirname: directory to read base images from
    :param image_dirname: directory to write generated images to
    :param csv_filename: CSV file to write generated images' parameter to
    :param overwrite: Whether to overwrite existing CSV file
    """
    csv_header = ['image_name', 'image_filename', 'patch_parameters', 'label']

    # prepare lists of available base images for image generation
    files = glob.glob(os.path.join(base_dirname, '*.jpg'))
    background_list = [f for f in files if 'background' in f]
    object_list = list()
    for f in files:
        for obj in objects:
            if obj in f:
                object_list.append(f)

    # write header to CSV file
    if overwrite:
        with gzip.open(csv_filename, 'w') as fp:
            csv_writer = csv.writer(fp)
            csv_writer.writerow(csv_header)

    start = time.time()
    # generate required number of images with custom properties
    # and write them to CSV file
    for idx in range(images):
        d = _assemble_image(background_list, object_list, image_dirname)
        with gzip.open(csv_filename, 'a') as fp:
            csv_writer = csv.writer(fp)
            csv_writer.writerow(d)

    print 'Wrote %i images in %.3fs.' % (images, time.time() - start)


def _assemble_image(bg_list, fg_list, image_dirname):
    data = ['bla1', 'bla2', ((1, 2, 3), (5, 6, 7), (8, 9, 10), (12, 13, 14)), (0, 1, 2, 3)]
    return data


def create_split(csv_filename, data_dirname, labels, images, split=(.7, .3)):
    """
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
    # fn = os.path.join(data_dirname, 'object_names.txt')
    # write_objects(filename=fn)

    assemble_data(objects, 10, base_dirname, image_dirname, csv_filename, overwrite=True)
    # create_split(csv_filename, data_dirname, 0, 10)

if __name__ == '__main__':
    main()

import os


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


def main():
    """ Assemble data for the 'synthetic demo data' set.

    Create a synthetic data set by placing object patches on top of background
    images. Apply a number of transformations to the images in order to
    'augment' the data set (cf. data augmentation).
    """
    script_dirname = os.path.abspath(os.path.dirname(__file__))
    data_dirname = os.path.abspath(os.path.join(script_dirname, '..', 'data',
                                                'synthetic_demo_data'))
    if not os.path.exists(data_dirname):
        os.makedirs(data_dirname)

    # write object names (label strings) to file,
    # their order in the file gives the label int's
    fn = os.path.join(data_dirname, 'object_names.txt')
    write_objects(filename=fn)

if __name__ == '__main__':
    main()

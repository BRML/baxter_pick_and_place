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

import cv2
import numpy as np
import os


def add_noise(image, snr=0.9):
    """ Add gaussian noise with a standard deviation given by the desired
    signal-to-noise ratio (std dev = 1 - snr)
    :param image: the image to modify
    :param snr: intended signal-to-noise ratio (in percent)
    [default = 0.9 => 10% noise]
    :return: the modified image
    """
    noise = np.zeros(shape=image.shape, dtype="uint8")
    cv2.randn(noise, mean=0, stddev=255*(1 - snr))
    return image + noise


def contrast_brightness(image, contrast, brightness):
    """ Modify contrast and brightness of an image. For each pixel, do
        p[c, r] = contrast*p[c, r] + brightness.
    Ensures that resulting pixel values remain in the data type's range.
    :param image: the image to modify
    :param contrast: the contrast multiplier
    :param brightness: the brightness additive term
    :return: the modified image
    """
    return cv2.convertScaleAbs(image, alpha=contrast, beta=brightness)


def gamma(image, gamma=1.0):
    """ Modify gamma values of image.
    Build a lookup table mapping the pixel values [0, 255] to [0, 1], adjust
    their gamma values and map them back to [0, 255]. Then apply the lookup
    table to the image.
    :param image: the image to modify
    :param gamma: gamma value to use
    :return: the modified image
    """
    inv_gamma = 1./gamma
    table = np.array([((i/255.0)**inv_gamma)*255
                      for i in np.arange(0, 256)]).astype('uint8')
    return cv2.LUT(image, table)


class Augmentation(object):

    def __init__(self):
        """ Data augmentation class. """
        self._contrast_max = 3.0
        self._brightness_min = -100
        self._brightness_max = 100
        self._gamma_max = 2.5
        self._snr = 0.9

    def augment_image(self, image):
        """ Perform random augmentation of image
        :param image: the image to modify
        :return: the modified image
        """
        print 'Augmented image with',
        # random change in contrast and brightness
        ctrst = np.random.random()*self._contrast_max
        brtns = np.random.randint(low=self._brightness_min,
                                  high=self._brightness_max)
        image = contrast_brightness(image, ctrst, brtns)
        print 'contrast=%.2f and brightness=%i' % (ctrst, brtns),

        # gamma modification
        gma = np.random.random()*self._gamma_max
        image = gamma(image, gma)
        print 'and gamma=%.2f' % gma,

        # Gaussian noise
        image = add_noise(image, self._snr)
        print 'and noise=%.2f.' % (1 - self._snr)

        return image

    @property
    def contrast(self):
        return self._contrast_max

    @contrast.setter
    def contrast(self, ctrst):
        """ Maximum contrast value for random contrast modification. """
        if ctrst < 0.0:
            raise ValueError("Invalid contrast value specified!")

        self._contrast_max = ctrst

    @property
    def brightness(self):
        return self._brightness_min, self._brightness_max

    @brightness.setter
    def brightness(self, brtns):
        """ Minimum and maximum brightness additive term (as a tuple) for
        random brightness modification.
        """
        if not isinstance(brtns, tuple):
            raise AttributeError("Invalid brightness specified!")
        if len(brtns) != 2:
            raise AttributeError("Invalid brightness specified!")
        if not brtns[0] < brtns[1]:
            raise ValueError("Minimum brightness must be lower than maximum brightness!")

        self._brightness_min, self._brightness_max = brtns

    @property
    def gamma(self):
        return self._gamma_max

    @gamma.setter
    def gamma(self, gma):
        """ Maximum gamma value for random gamma modification. """
        if gma < 0.0:
            raise ValueError("Invalid gamma value specified!")

        self._gamma_max = gma

    @property
    def snr(self):
        return self._snr

    @snr.setter
    def snr(self, ratio):
        """ Signal-to-noise ratio for additive Gaussian noise (in percent),
        where snr = 1 - noise [%].
        """
        if not 0.0 <= ratio <= 1.0:
            raise ValueError("Invalid signal-to-noise ratio specified!")

        self._snr = ratio


def main():
    """ Demonstrate data augmentation.

    Apply a number of transformations to an image in order to 'augment' a
    data set.
    """
    src_dirname = os.path.abspath(os.path.dirname(__file__))
    base_dirname = os.path.abspath(os.path.join(src_dirname, '..', '..',
                                                'data', 'sdd'))
    filename = os.path.join(base_dirname, 'background_1.jpg')
    orig = cv2.imread(filename)
    h, w, = orig.shape[:2]

    aug = Augmentation()
    aug.contrast = 2.0
    aug.brightness = -50, 50
    aug.gamma = 1.5
    aug.snr = 0.9
    img = aug.augment_image(orig)

    cv2.imshow('original', cv2.resize(orig, (w/2, h/2)))
    cv2.imshow('augmented', cv2.resize(img, (w/2, h/2)))
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

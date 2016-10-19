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

from demo import demo_remove_default_loghandler
from hardware import (
    cam_remove_default_loghandler,
    baxter_remove_default_loghandler,
    kinect_remove_default_loghandler
)
from servoing import servo_remove_default_loghandler
from simulation import env_remove_default_loghandler
from vision import mnc_remove_default_loghandler


def redirect_logger(fname='', level=logging.INFO):
    """Redirect logging output of all modules to a common stream handler and
    optional file handler with common format and log level.

    :param fname: The file name for the optional log file handler. If None,
        no file handler is created.
    :param level: The log level to use.
    :return:
    """
    handles = ['demo', 'cam', 'baxter', 'kinect', 'servo', 'env', 'mnc']

    # remove default log handlers
    for handle in handles:
        eval('{}_remove_default_loghandler()'.format(handle))

    handlers = get_default_handler(filename=fname, level=level)
    for handler in handlers:
        for handle in handles:
            logging.getLogger(handle).addHandler(hdlr=handler)


def get_default_handler(filename='', level=logging.INFO):
    h = list()

    sh = logging.StreamHandler()
    sh.setLevel(level=level)
    fmt = '[%(name)s][%(levelname)s] %(message)s'
    sh.setFormatter(fmt=logging.Formatter(fmt=fmt))
    h.append(sh)

    if isinstance(filename, str) and len(filename) > 0:
        fh = logging.FileHandler(filename=filename, mode='a')
        fh.setLevel(level=level)
        fmt = '[%(name)s][%(levelname)s][%(asctime)-15s] %(message)s'
        fh.setFormatter(fmt=logging.Formatter(fmt=fmt))
        h.append(fh)

    return h

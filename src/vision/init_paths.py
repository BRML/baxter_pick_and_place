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

import os
import sys


def add_path(path):
    if path not in sys.path:
        sys.path.insert(0, path)


develop_dir = os.path.expanduser('~/software')


def set_up_faster_rcnn():
    """Set up python paths for faster R-CNN."""
    # Add Caffe to PYTHONPATH
    caffe_path = os.path.join(develop_dir,
                              'py-faster-rcnn', 'caffe-fast-rcnn', 'python')
    add_path(caffe_path)

    # Add lib to PYTHONPATH
    lib_path = os.path.join(develop_dir, 'py-faster-rcnn', 'lib')
    add_path(lib_path)


def set_up_mnc():
    """Set up python paths for MNC."""
    # Add Caffe to PYTHONPATH
    caffe_path = os.path.join(develop_dir,
                              'mnc', 'caffe-mnc', 'python')
    add_path(caffe_path)

    # Add lib to PYTHONPATH
    lib_path = os.path.join(develop_dir, 'mnc', 'lib')
    add_path(lib_path)

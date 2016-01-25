#!/usr/bin/env python

import numpy as np
import os
import sys

caffe_root = u'/home/baxter/software/caffe/'
sys.path.insert(0, caffe_root + 'python')
import caffe

if not os.path.isfile(caffe_root + 'models/bvlc_reference_caffenet/bvlc_reference_caffenet.caffemodel'):
    print("Please download pre-trained CaffeNet model by executing",
          "scripts/download_model_binary.py models/bvlc_reference_caffenet")

# set Caffe to CPU mode and load the net set to test phase for inference
caffe.set_mode_cpu()
net = caffe.Net(caffe_root + u'models/bvlc_reference_caffenet/deploy.prototxt',
                caffe_root + u'models/bvlc_reference_caffenet/bvlc_reference_caffenet.caffemodel',
                caffe.TEST)
# configure input pre-processing
transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape})
transformer.set_transpose('data', (2,0,1))
transformer.set_mean('data', np.load(caffe_root + 'python/caffe/imagenet/ilsvrc_2012_mean.npy').mean(1).mean(1)) # mean pixel
transformer.set_raw_scale('data', 255)  # the reference model operates on images in [0,255] range instead of [0,1]
transformer.set_channel_swap('data', (2,1,0))  # the reference model has channels in BGR order instead of RGB

batch_size=50
net.blobs['data'].reshape(batch_size, 3, 227, 227)

# feed in image and classify with a forward pass
net.blobs['data'].data[...] = transformer.preprocess('data', caffe.io.load_image(caffe_root + 'examples/images/cat.jpg'))
out = net.forward()
print("Predicted class is #{}.".format(out['prob'][0].argmax()))

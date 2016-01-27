#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import time

caffe_root = '/home/baxter/software/caffe/'
model_root = caffe_root + 'models/bvlc_reference_caffenet/'
model_file = model_root + 'bvlc_reference_caffenet.caffemodel'
model_proto = model_root + 'deploy.prototxt'
# we do not need this if run from console, but for pycharm
sys.path.insert(0, caffe_root + 'python')
import caffe


if not os.path.isfile(model_file):
    print("Please download pre-trained CaffeNet model by executing",
          "scripts/download_model_binary.py models/bvlc_reference_caffenet")

# set Caffe to CPU mode and load the net set to test phase for inference
caffe.set_mode_cpu()
net = caffe.Net(model_proto, model_file, caffe.TEST)
# configure input pre-processing
transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape})
transformer.set_transpose('data', (2, 0, 1))
mean_file = caffe_root + 'python/caffe/imagenet/ilsvrc_2012_mean.npy'
transformer.set_mean('data', np.load(mean_file).mean(1).mean(1))  # mean pixel
transformer.set_raw_scale('data', 255)  # the reference model operates on images in [0,255] range instead of [0,1]
transformer.set_channel_swap('data', (2, 1, 0))  # the reference model has channels in BGR order instead of RGB

batch_size = 1
net.blobs['data'].reshape(batch_size, 3, 227, 227)

# feed in image and classify with a forward pass
example_image = caffe_root + 'examples/images/cat.jpg'
net.blobs['data'].data[...] = transformer.preprocess('data', caffe.io.load_image(example_image))
out = net.forward()
print("Predicted class is #{}.".format(out['prob'][0].argmax()))

# plt.imshow(transformer.deprocess('data', net.blobs['data'].data[0]))
# plt.show()

# load labels
imagenet_labels_filename = caffe_root + 'data/ilsvrc12/synset_words.txt'
labels = np.loadtxt(imagenet_labels_filename, str, delimiter='\t')

# sort top k predictions from softmax output
top_k = net.blobs['prob'].data[0].flatten().argsort()[-1:-6:-1]
print labels[top_k]

net.forward()  # call once for allocation
k = 3
duration = np.zeros(k)
for i in range(k):
    start = time.time()
    net.forward()
    duration[i] = time.time() - start
print '%i loops\n average: %.2fs per loop,\n best: %.2fs.' % (k,
                                                              duration.mean(),
                                                              duration.min())

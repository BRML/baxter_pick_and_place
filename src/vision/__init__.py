# -*- coding: utf-8 -*-

"""Module for computer vision related software components."""

# There is a conflict when multiple versions of Caffe are on the path!
# We circumvent this conflict by only loading/using the module we actually are
# going to use.
# Faster Region-based Convolutional Networks for object detection
# from detection import ObjectDetection
# Fully convolutional networks for object detection
from detection2 import ObjectDetection
# Instance-aware Semantic Segmentation via Multi-task Network Cascades
# from segmentation import ObjectSegmentation

from visualization_utils import (
    draw_detection,
    draw_rroi,
    mask_to_rroi,
    color_difference
)

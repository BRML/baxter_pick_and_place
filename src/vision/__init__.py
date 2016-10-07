# -*- coding: utf-8 -*-

"""Module for computer vision related software components."""

# There is a conflict when both versions of Caffe are on the path---circumvent
# it by not loading/using the faster R-CNN at all.
# from detection import (
#     ObjectDetection,
#     remove_default_loghandler as rcnn_remove_default_loghandler
# )

from segmentation import (
    ObjectSegmentation,
    remove_default_loghandler as mnc_remove_default_loghandler
)

from visualization_utils import (
    draw_rroi,
    mask_to_rroi,
    color_difference
)

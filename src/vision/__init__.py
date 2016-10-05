# -*- coding: utf-8 -*-

"""Module for computer vision related software components."""

from detection import (
    ObjectDetection,
    remove_default_loghandler as rcnn_remove_default_loghandler
)

from segmentation import (
    ObjectSegmentation,
    remove_default_loghandler as mnc_remove_default_loghandler
)

from visualization_utils import (
    draw_rroi,
    mask_to_rroi,
    color_difference
)

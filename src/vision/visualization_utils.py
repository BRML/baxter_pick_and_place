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


import cv2


# Set colors for visualisation of detections
red = (0, 0, 255)  # BGR
yellow = (0, 255, 255)
black = (0, 0, 0)
white = (255, 255, 255)


def textbox(image, text, org, font_face, font_scale, thickness, color, color_box):
    """Draw a filled box with text placed on top of it.

    :param image: The image to draw on.
    :param text: The text to put there.
    :param org: The origin of the text (bottom left corner) (x, y).
    :param font_face: The font to use.
    :param font_scale: The scale for the font.
    :param thickness: The thickness of the font.
    :param color: The color of the font (BGR).
    :param color_box: The color of the box (BGR).
    :return:
    """
    (w, h), _ = cv2.getTextSize(text=text,
                                fontFace=font_face, fontScale=font_scale,
                                thickness=thickness)
    ox, oy = [int(o) for o in org]
    cv2.rectangle(image, pt1=(ox - 2, oy + 2), pt2=(ox + 2 + w, oy - 2 - h),
                  color=color_box, thickness=cv2.cv.CV_FILLED)
    cv2.putText(image, text=text, org=(ox, oy),
                fontFace=font_face, fontScale=font_scale,
                thickness=thickness, color=color)


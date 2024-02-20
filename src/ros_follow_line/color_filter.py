"""
color_filter.py
"""

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
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


__authors__ = "Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2024 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"


import cv2
import numpy as np


def color_filter(img_rgb: np.ndarray, color_lower: list, color_upper: list,
                 min_thresh: int = 60, max_thresh: int = 255) -> np.ndarray:
    """Perform color filter over a RGB img

    :param img_rgb: original RGB image
    :param lower_threshold: Color lower threshold in HSV
    :param upper_threshold: Color upper threshold in HSV
    :param min_thresh: Minimum binary threshold
    :param max_thresh: Maximum binary threshold
    :return: Binary image
    """
    hsv_img = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2HSV)
    mask_img = cv2.inRange(hsv_img, np.array(
        color_lower), np.array(color_upper))
    color_filtered_img = cv2.bitwise_and(
        img_rgb, img_rgb, mask=mask_img)
    grayscaled_img = cv2.cvtColor(color_filtered_img, cv2.COLOR_BGR2GRAY)
    blurred_img = cv2.GaussianBlur(grayscaled_img, (5, 5), 0)
    binary_img = cv2.threshold(
        blurred_img, min_thresh, max_thresh, cv2.THRESH_BINARY)[1]
    return binary_img


def red_filter(img_rgb: np.ndarray) -> np.ndarray:
    """_summary_

    :param img_rgb: Original RGB image
    :return: Binary image filtered
    """
    return color_filter(img_rgb, [0, 200, 200], [20, 255, 255], 60, 255)


def get_centroid(binary_img: np.ndarray) -> tuple:
    """Get centroid of binary image

    :param binary_img: binary image
    :return: centroid (cx, cy)
    """
    # Get contours of binary image
    contours, _ = cv2.findContours(
        binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Get centroid of the bigest mask
    if len(contours) > 0:
        bigest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(bigest_contour) > 400:
            M = cv2.moments(bigest_contour)
            if M["m00"] != 0:
                # calculate x,y coordinate of center
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                centroid = (cX, cY)
                return centroid
    return None

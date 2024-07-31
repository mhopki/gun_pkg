#!/usr/bin/env python

import cv2

# Load input and reference images
input_img = cv2.imread('/home/malakhi/catkin_ws/src/gun_pkg/maps/map.png')
ref_img = cv2.imread('/home/malakhi/catkin_ws/src/gun_pkg/maps/projected_map.png')

# Get size of reference image
ref_height, ref_width, _ = ref_img.shape
in_h, in_w, _ = input_img.shape
in_hm = in_h // 2
in_wm = in_w // 2
ref_hm = ref_height // 2
ref_wm = ref_width // 2
x_off = -10
y_off = 15

# Crop input image to size of reference image
input_cropped = input_img[in_hm - ref_hm + y_off: in_hm - ref_hm + ref_height + y_off, in_wm - ref_wm + x_off:in_wm - ref_wm + ref_width + x_off]

# Save cropped input image
cv2.imwrite('/home/malakhi/catkin_ws/src/gun_pkg/maps/new_map.png', input_cropped)

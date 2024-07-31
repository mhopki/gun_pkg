#!/usr/bin/env python

import cv2
import numpy as np
import os
import yaml

map1_path = '/home/malakhi/catkin_ws/src/gun_pkg/maps/yaml_map.yaml'
map2_path = '/home/malakhi/catkin_ws/src/gun_pkg/maps/yaml_projected_map.yaml'

with open(map1_path, 'r') as f:
    map1_data = yaml.load(f)
with open(map2_path, 'r') as f:
    map2_data = yaml.load(f)

# Compute transformation matrices and sizes
res = map1_data['resolution']
origin1 = np.array([map1_data['origin'][0], map1_data['origin'][1], 1])
origin2 = np.array([map2_data['origin'][0], map2_data['origin'][1], 1])
size1 = np.array([map1_data['image_height'], map1_data['image_width']])
size2 = np.array([map2_data['image_height'], map2_data['image_width']])
offset = np.round((origin2 - origin1)[:2] / res).astype(np.int)

# Load input and reference images
input_img = cv2.imread(os.path.join(os.path.dirname(map1_path), map1_data['image'] + '.png'))#'/home/malakhi/catkin_ws/src/gun_pkg/maps/map.png')
ref_img = cv2.imread(os.path.join(os.path.dirname(map2_path), map2_data['image'] + '.png'))#'/home/malakhi/catkin_ws/src/gun_pkg/maps/projected_map.png')

# Get size of reference image
ref_height, ref_width, _ = ref_img.shape
in_h, in_w, _ = input_img.shape
in_hm = in_h // 2
in_wm = in_w // 2
ref_hm = ref_height // 2
ref_wm = ref_width // 2
x_off = -int(in_wm//origin1[0])# + in_wm)
y_off = 0 - int(round(origin2[1]))#int(in_hm//origin1[1]+22)# + in_hm)
#x_off = 2000//100#20
#y_off = -2000//100
print(in_hm, in_wm, ref_hm, ref_wm, x_off, y_off)

# Crop input image to size of reference image
input_cropped = input_img[in_hm - ref_hm + y_off: in_hm - ref_hm + ref_height + y_off, in_wm - ref_wm + x_off:in_wm - ref_wm + ref_width + x_off]

# Save cropped input image
cv2.imwrite('/home/malakhi/catkin_ws/src/gun_pkg/maps/new_map.png', input_cropped)

#!/usr/bin/env python

import numpy as np
import cv2
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

trans1 = np.array([[1, 0, -origin1[0]],
                   [0, 1, -origin1[1]],
                   [0, 0, 1]])

trans2 = np.array([[1, 0, -origin2[0]],
                   [0, 1, -origin2[1]],
                   [0, 0, 1]])

rot1 = np.array([[np.cos(map1_data['origin'][2]), -np.sin(map1_data['origin'][2]), 0],
                 [np.sin(map1_data['origin'][2]), np.cos(map1_data['origin'][2]), 0],
                 [0, 0, 1]])

rot2 = np.array([[np.cos(map2_data['origin'][2]), -np.sin(map2_data['origin'][2]), 0],
                 [np.sin(map2_data['origin'][2]), np.cos(map2_data['origin'][2]), 0],
                 [0, 0, 1]])

scale_diff = np.array([[res, 0, 0],
                       [0, res, 0],
                       [0, 0, 1]])

rot_scale_diff = np.dot(scale_diff, rot2.dot(np.linalg.inv(rot1)))
rot_scale_diff[0, 2] += offset[1] + size1[1]
rot_scale_diff[1, 2] += offset[0] + size1[0]

# Load and transform the maps
map1_img = cv2.imread(os.path.join(os.path.dirname(map1_path), map1_data['image'] + '.png'))
map2_img = cv2.imread(os.path.join(os.path.dirname(map2_path), map2_data['image'] + '.png'))

map1_warped = cv2.warpAffine(map1_img, trans1[:2], tuple(size1[::-1]))
map2_warped = cv2.warpAffine(map2_img, trans2[:2], tuple(size2[::-1]))

merged_size = (size2 + abs(offset)).astype(np.int)
map1_warped = cv2.warpAffine(map1_warped, rot_scale_diff[:2], tuple(merged_size[::-1]))

# Create merged map and save/display
merged_size = (size2 + abs(offset)).astype(np.int)
merged = np.zeros((merged_size[0], merged_size[1], 3), np.uint8)
merged[:size1[0], :size1[1]] = map1_warped
merged[offset[0]:offset[0]+size2[0], offset[1]:offset[1]+size2[1]] = map2_warped


# Display the merged image
cv2.imshow('Merged Map', merged)
cv2.waitKey(0)

# Save the merged image to a file
cv2.imwrite('/home/malakhi/catkin_ws/src/gun_pkg/maps/yaml_merged_map.png', merged)

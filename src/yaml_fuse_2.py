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

res = map1_data['resolution']
origin1 = np.array([map1_data['origin'][0], map1_data['origin'][1], 1])
origin2 = np.array([map2_data['origin'][0], map2_data['origin'][1], 1])
size1 = np.array([map1_data['image_height'], map1_data['image_width']])
size2 = np.array([map2_data['image_height'], map2_data['image_width']])

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

size_diff = np.abs(size1 - size2)
size_max = np.maximum(size1, size2)

max_offset = np.round(size_diff * res / 2).astype(np.int)
start1 = max_offset
end1 = max_offset + size1
start2 = max_offset
end2 = max_offset + size2

map1_img = cv2.imread(os.path.join(os.path.dirname(map1_path), map1_data['image']+ '.png'))
map2_img = cv2.imread(os.path.join(os.path.dirname(map2_path), map2_data['image']+ '.png'))

#print(os.path.join(os.path.dirname(map1_path), map1_data['image']))
#print(map1_img)
#print(map2_img)

if map1_img.shape[0] <= 0 or map1_img.shape[1] <= 0:
    print("Error: invalid image dimensions for map1")

if map2_img.shape[0] <= 0 or map2_img.shape[1] <= 0:
    print("Error: invalid image dimensions for map2")

map1_warped = cv2.warpAffine(map1_img, trans1[:2], tuple(size_max))
map2_warped = cv2.warpAffine(map2_img, trans2[:2], tuple(size_max))

rot_diff = np.dot(rot2, np.linalg.inv(rot1))
map1_warped = cv2.warpAffine(map1_warped, rot_diff[:2], tuple(size_max))

merged = np.zeros((size_max[0], size_max[1], 3), np.uint8)
merged[start1[0]:end1[0], start1[1]:end1[1]] = map1_warped[start1[0]:end1[0], start1[1]:end1[1]]
merged[start2[0]:end2[0], start2[1]:end2[1]] = map2_warped[start2[0]:end2[0], start2[1]:end2[1]]

cv2.imwrite('/home/malakhi/catkin_ws/src/gun_pkg/maps/yaml_merged_map.png', merged)

#!/usr/bin/env python

import cv2

# Load the first image
img2 = cv2.imread('/home/malakhi/catkin_ws/src/gun_pkg/maps/yaml_map.png')

# Load the second image
img1 = cv2.imread('/home/malakhi/catkin_ws/src/gun_pkg/maps/yaml_projected_map.png')

# Resize the second image to match the size of the first image
img2_resized = cv2.resize(img2, (img1.shape[1], img1.shape[0]))

# Overlay the second image on top of the first image using alpha blending
alpha = 0.5
overlay = cv2.addWeighted(img1, alpha, img2_resized, 1 - alpha, 0)

cv2.imwrite("/home/malakhi/catkin_ws/src/gun_pkg/maps/yaml_overlay_map.png",overlay)

# Display the overlayed image
#cv2.imshow('Overlay', overlay)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

import cv2
import numpy as np

# Load the maps
map1 = cv2.imread('map1.png', cv2.IMREAD_GRAYSCALE)
map2 = cv2.imread('map2.png', cv2.IMREAD_GRAYSCALE)

# Threshold the maps to convert them to binary images
_, map1_binary = cv2.threshold(map1, 127, 255, cv2.THRESH_BINARY)
_, map2_binary = cv2.threshold(map2, 127, 255, cv2.THRESH_BINARY)

# Find the differences between the maps
diff = cv2.absdiff(map1_binary, map2_binary)

# Label the differences as open or occupied
open_space = np.zeros_like(diff)
open_space[diff == 255] = 255

occupied_space = np.zeros_like(diff)
occupied_space[diff == 0] = 255

# Display the labeled differences
cv2.imshow('Open Space', open_space)
cv2.imshow('Occupied Space', occupied_space)
cv2.waitKey(0)
cv2.destroyAllWindows()

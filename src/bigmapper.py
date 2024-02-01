#!/usr/bin/env python

import cv2
import rospy
from nav_msgs.msg import OccupancyGrid

# Define the desired size of the map
MAP_SIZE = 4000

# Initialize the OccupancyGrid subscriber
proj_map_sub = rospy.Subscriber('/projected_map', OccupancyGrid, proj_map_callback)

def proj_map_callback(msg):
    # Convert the OccupancyGrid to a numpy array
    proj_map = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

    # Resize the projected_map to match the size of the map
    resized_proj_map = cv2.resize(proj_map, (MAP_SIZE, MAP_SIZE), interpolation=cv2.INTER_NEAREST)

    # Do further processing on the resized_proj_map
    # ...

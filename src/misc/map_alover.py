#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from tf2_geometry_msgs import PointStamped
from tf.transformations import quaternion_matrix

rospy.init_node('map_alignment')

# Set up TF2 buffer and listener
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

# Subscribe to the two map topics
map1_topic = '/map'
map2_topic = '/projected_map'
map1 = None
map2 = None

def map1_callback(data):
    global map1
    map1 = data

def map2_callback(data):
    global map2
    map2 = data

rospy.Subscriber(map1_topic, OccupancyGrid, map1_callback)
rospy.Subscriber(map2_topic, OccupancyGrid, map2_callback)

# Wait for both maps to be received
while map1 is None or map2 is None:
    rospy.sleep(0.1)

# Get transformation between the two maps
try:
    trans = tfBuffer.lookup_transform(map1.header.frame_id, map2.header.frame_id, rospy.Time())
except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    rospy.logerr('Failed to get transform between maps!')
    exit()

# Get transformation as OpenCV matrix
#quat = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
#rot = quaternion_matrix(quat)[:3, :3]
#rot, _ = cv2.Rodrigues(rot_mat)
#rot = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
rot = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z])
R, _ = cv2.Rodrigues(rot)
t = np.array([trans.transform.translation.x, trans.transform.translation.y])
T = np.vstack([np.hstack([R, t.reshape(2, 1)]), np.array([0, 0, 1])])

# Transform map1 to align with map2
map1_transformed = cv2.warpAffine(map1.data.reshape(map1.info.height, map1.info.width).astype(np.float32), T[:2], (map2.info.width, map2.info.height))

# Overlay the two maps
map_overlayed = cv2.addWeighted(map1_transformed, 0.5, map2.data.reshape(map2.info.height, map2.info.width).astype(np.float32), 0.5, 0)

# Save the overlayed map as an image
cv2.imwrite('overlayed_map.png', map_overlayed)
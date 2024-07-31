#!/usr/bin/env python

import rospy
import cv2
from nav_msgs.msg import OccupancyGrid
import numpy as np

# List of map topics to save
map_topics = ['/map', '/projected_map']

def save_map_to_png(map_topic):
    map_msg = rospy.wait_for_message(map_topic, OccupancyGrid)
    map_data = list(map_msg.data)
    map_width = map_msg.info.width
    map_height = map_msg.info.height
    map_img = [255 - int(round(x * 255. / 100.)) for x in map_data]
    map_img = np.array(map_img).reshape((map_height, map_width))
    map_img = cv2.convertScaleAbs(map_img)
    map_img = cv2.cvtColor(map_img, cv2.COLOR_GRAY2RGB)


    cv2.imwrite("/home/malakhi/catkin_ws/src/gun_pkg/maps/" + map_topic.replace('/', '') + '.png', map_img)

if __name__ == '__main__':
    rospy.init_node('map_saver')
    for map_topic in map_topics:
        save_map_to_png(map_topic)

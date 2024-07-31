#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

class MapToImage:
    def __init__(self):
        self.map_data = None
        self.map_metadata = None
        self.bridge = CvBridge()

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.image_pub = rospy.Publisher('/map_image', Image, queue_size=10)

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.uint8).reshape(msg.info.height, msg.info.width, -1)
        self.map_metadata = msg.info

    def save_map_as_image(self, filename):
        if self.map_data is not None and self.map_metadata is not None:
            map_resolution = self.map_metadata.resolution
            map_origin = self.map_metadata.origin.position
            map_image = np.flipud(np.rot90(self.map_data))
            cv2.imwrite(filename, map_image)

    def publish_image_topic(self):
        if self.map_data is not None and self.map_metadata is not None:
            map_resolution = self.map_metadata.resolution
            map_origin = self.map_metadata.origin.position
            map_image = np.flipud(np.rot90(self.map_data))
            map_image_msg = self.bridge.cv2_to_imgmsg(map_image, encoding="passthrough")
            map_image_msg.header.stamp = rospy.Time.now()
            self.image_pub.publish(map_image_msg)

if __name__ == '__main__':
    rospy.init_node('map_to_image', anonymous=True)
    map_to_image = MapToImage()

    # Save map as an image file
    map_to_image.save_map_as_image('map_image.png')

    # Publish map as an image topic
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        map_to_image.publish_image_topic()
        rate.sleep()

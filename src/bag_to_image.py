#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2
import struct
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import pyoctree
import rospy
from octomap_msgs.msg import Octomap
from octomap_msgs.srv import BoundingBoxQuery
from octomap_msgs.srv import GetOctomap
from sensor_msgs.msg import Range

class BagToImage:
    def __init__(self):
        rospy.init_node('depth_to_pointcloud_converter', anonymous=True)
        self.bridge = CvBridge()

        # Subscriber for depth image and camera info
        #rospy.Subscriber('/depth_gray_image', Image, self.image_callback)
        #rospy.Subscriber('/yolov7/yolov7/visualization', Image, self.image_callback)
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        # Create a Rate object to control the loop frequency
        self.rate = rospy.Rate(10)  # 10 Hz

        # Main loop
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def image_callback(self, img_msg):

        # Convert depth image to numpy array
        image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

        #SAVE IMAGE TO FILE
        #"""
        save_path = '/home/malakhi/Pictures/CNN/Bagfiles/1'
        # Save the image as JPEG
        timestamp_str = rospy.Time.now().to_sec()  # Convert timestamp to seconds
        image_filename = f"{save_path}/captured_image_{timestamp_str:.6f}.jpg"
        cv2.imwrite(image_filename, image)
        rospy.loginfo("Image saved as %s", image_filename)
        print("saved: ", image_filename)
        #"""

if __name__ == '__main__':
    try:
        converter = BagToImage()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass

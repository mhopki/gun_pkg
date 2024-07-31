#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray

global cam_type, cam_fish
cam_type = 1
cam_fish = 1

def image_callback(msg):
    bridge = CvBridge()
    global cam_type

    # Convert the depth image to a numpy array
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # Create a depth mask for the circle
    y, x = np.ogrid[:depth_image.shape[0], :depth_image.shape[1]]

    # Create a grayscale image (single-channel)
    #Realsensed435i alpha=0.0052, size = 640,480
    if cam_type == 0:
        depth_gray = cv2.convertScaleAbs(depth_image, alpha=0.0052)
    elif cam_type == 1:
        depth_gray = cv2.convertScaleAbs(depth_image, alpha=0.99) #0.03 # Adjust alpha as needed, makes distance gap bigger
        depth_gray = cv2.GaussianBlur(depth_gray, (15, 15), 0) #((33, 33), 0)
        #depth_gray = cv2.medianBlur(depth_gray, 31) #61
        #depth_gray = cv2.bitwise_not(depth_gray)

    if cam_fish == 1:
        #TOF FISHEYE
        radius = 95
        mask = (x - depth_image.shape[1] // 2) ** 2 + (y - depth_image.shape[0] // 2) ** 2 >= radius ** 2
        dp = 0 if cam_type == 0 else 255
        depth_gray[mask] = dp

    depth_gray_msg = bridge.cv2_to_imgmsg(depth_gray, encoding="mono8")
    depth_gray_pub.publish(depth_gray_msg)

def main():
    rospy.init_node('square_detection_node')
    
    # Create publishers for depth_gray, depth_color, and thresholded images
    global depth_gray_pub, depth_color_pub, thresholded_pub, depth_image_pub, window_pub
    global cam_type
    depth_gray_pub = rospy.Publisher('depth_gray_image', Image, queue_size=1)

    if cam_type == 0:
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, image_callback)
    elif cam_type == 1:
        rospy.Subscriber('/dragonfly26/tof/voxl_depth_image_raw', Image, image_callback)
    #rospy.spin()

if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

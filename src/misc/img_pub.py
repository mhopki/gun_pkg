#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def create_image():
    # Image dimensions
    width, height = 1280, 720

    # Create a white image
    image = np.ones((height, width, 3), np.uint8) * 0  # White image

    # Create a black rectangle in the center
    rect_width, rect_height = 60, 40
    rect_color = (255, 255, 255)  # Black color
    x1 = (width - rect_width) // 2
    y1 = (height - rect_height) // 2
    x2 = x1 + rect_width
    y2 = y1 + rect_height
    cv2.rectangle(image, (x1, y1), (x2, y2), rect_color, -1)  # Filled rectangle

    return image

def main():
    rospy.init_node('image_publisher', anonymous=True)
    image_pub = rospy.Publisher('image_topic', Image, queue_size=1)
    bridge = CvBridge()

    rate = rospy.Rate(10)  # Publish rate (e.g., 10 Hz)
    go = 5

    while not rospy.is_shutdown():
        image = create_image()

        # Publish the image
        image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        #if go > 0:
        image_pub.publish(image_msg)
        #    go -= 1
        #print("pub")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

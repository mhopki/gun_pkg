#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    # Crop the image
    crop = data.height // 2 - 64
    #H=480, W=848
    cropped_image = cv_image[0+crop:data.height-crop, :]
    cropped_msg = bridge.cv2_to_imgmsg(cropped_image, encoding='passthrough')
    cropped_msg.header = data.header

    # Convert the cropped image back to ROS format
    """try:
        cropped_msg = bridge.cv2_to_imgmsg(cropped_image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)"""
    
    # Publish the cropped image to the image topic
    pub.publish(cropped_msg)

if __name__ == '__main__':
    rospy.init_node('image_cropper', anonymous=True)
    pub = rospy.Publisher('/camera/depth/image_rect_raw/crop', Image, queue_size=10)
    rospy.Subscriber('/camera/depth/image_rect_raw', Image, callback)

    rospy.spin()

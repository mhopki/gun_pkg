#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def image_callback(msg):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Threshold the image to isolate black areas
    _, thresholded = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)

    thresholded_msg = bridge.cv2_to_imgmsg(thresholded, encoding="mono8")
    thresholded_pub.publish(thresholded_msg)

    # Find contours in the thresholded image
    _, contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Check if any contours represent a square
    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        print(approx)

        # If the polygon has four vertices, it's likely a square
        if len(approx) == 4:
            # Draw a bounding rectangle around the square
            x, y, w, h = cv2.boundingRect(approx)
            if w > 100 and h > 100:
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 1)
                cv2.putText(image, "Square", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
                print("Square found")
                break
    else:
        print("No square")

    # Publish the depth_gray image
    depth_gray_msg = bridge.cv2_to_imgmsg(gray, encoding="mono8")
    depth_gray_pub.publish(depth_gray_msg)

    # Publish the depth_color image
    depth_color_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
    depth_color_pub.publish(depth_color_msg)

    # Display the annotated image
    cv2.imshow("Square Detection", image)
    cv2.waitKey(3)

def main():
    rospy.init_node('square_detection_node')

    global depth_gray_pub, depth_color_pub, thresholded_pub
    depth_gray_pub = rospy.Publisher('depth_gray_image', Image, queue_size=1)
    depth_color_pub = rospy.Publisher('depth_color_image', Image, queue_size=1)
    thresholded_pub = rospy.Publisher('thresholded_image', Image, queue_size=1)

    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

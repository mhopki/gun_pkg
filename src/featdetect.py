#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageOverlay:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub1 = rospy.Subscriber('/image1', Image, self.image_callback1)
        self.image_sub2 = rospy.Subscriber('/image2', Image, self.image_callback2)
        self.image_pub = rospy.Publisher('/image_overlay', Image, queue_size=10)

        self.img1 = None
        self.img2 = None
        self.kp1 = None
        self.des1 = None
        self.kp2 = None
        self.des2 = None

        self.sift = cv2.xfeatures2d.SIFT_create()
        self.bf = cv2.BFMatcher()

    def image_callback1(self, data):
        self.img1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(self.img1, cv2.COLOR_BGR2GRAY)
        self.kp1, self.des1 = self.sift.detectAndCompute(gray, None)

    def image_callback2(self, data):
        self.img2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(self.img2, cv2.COLOR_BGR2GRAY)
        self.kp2, self.des2 = self.sift.detectAndCompute(gray, None)
        
        if self.img1 is not None:
            matches = self.bf.match(self.des1, self.des2)
            matches = sorted(matches, key=lambda x:x.distance)
            good_matches = matches[:10]
            img3 = cv2.drawMatches(self.img1,self.kp1,self.img2,self.kp2,good_matches,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

            # Overlay images
            h, w, _ = img3.shape
            overlay = np.zeros((h, w, 3), np.uint8)
            overlay[0:h, 0:w] = img3
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(overlay, "bgr8"))

def main():
    rospy.init_node('image_overlay')
    ic = ImageOverlay()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
